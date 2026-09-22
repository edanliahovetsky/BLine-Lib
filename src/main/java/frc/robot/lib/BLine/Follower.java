package frc.robot.lib.BLine;

import org.wpilib.math.util.MathUtil;
import org.wpilib.util.Pair;
import org.wpilib.math.controller.PIDController;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.system.Timer;
import frc.robot.lib.BLine.Path.PathElement;
import frc.robot.lib.BLine.Path.PathElementConstraint;
import frc.robot.lib.BLine.Path.EventTrigger;
import frc.robot.lib.BLine.Path.RotationTarget;
import frc.robot.lib.BLine.Path.RotationTargetConstraint;
import frc.robot.lib.BLine.Path.TranslationTarget;
import frc.robot.lib.BLine.Path.TranslationTargetConstraint;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import java.util.function.BooleanSupplier;
import java.util.Objects;


/**
 * Framework-independent path-following state and control calculations.
 *
 * <p>The command adapters own scheduler requirements and delegate execution here. Keeping the
 * mutable traversal state in one instance lets both command frameworks use the same controller.
 */
final class Follower {
    /**
     * Determines how a rotation override interacts with BLine's normal constraints.
     */
    enum RotationOverrideBehavior {
        /**
         * The supplied omega replaces the rotation PID output before BLine applies its
         * existing rotational velocity and acceleration limits.
         */
        RESPECT_CONSTRAINTS,

        /**
         * BLine still rate-limits translation, but the supplied omega is restored after
         * path-follower limiting so the caller owns the final rotational command.
         */
        BYPASS_CONSTRAINTS
    }

    private static final java.util.logging.Logger logger = java.util.logging.Logger.getLogger(Follower.class.getName());
    // Explicit sentinel for "no active rotation target selected".
    private static final int NO_ACTIVE_ROTATION_INDEX = -1;
    // Defaults to FPGA-backed time but is overrideable in tests for deterministic simulation.
    private static Supplier<Double> timestampSupplier = Timer::getTimestamp;
    private static Consumer<Pair<String, Pose2d>> poseLoggingConsumer = value -> {};
    private static Consumer<Pair<String, Translation2d[]>> translationListLoggingConsumer = value -> {};
    private static Consumer<Pair<String, Double>> doubleLoggingConsumer = value -> {};
    private static Consumer<Pair<String, Boolean>> booleanLoggingConsumer = value -> {};
    private org.wpilib.telemetry.TelemetryTable telemetry;
    private final PendingEvents events;
    private PendingEvents.Execution eventExecution;
    private static volatile DoubleSupplier rotationOverrideSupplier = null;
    private static volatile RotationOverrideBehavior rotationOverrideBehavior =
        RotationOverrideBehavior.BYPASS_CONSTRAINTS;

    private void logDouble(String key, double value) {
        if (telemetry != null) telemetry.log(key, value);
        doubleLoggingConsumer.accept(new Pair<>(key, value));
    }

    private void logBoolean(String key, boolean value) {
        if (telemetry != null) telemetry.log(key, value);
        booleanLoggingConsumer.accept(new Pair<>(key, value));
    }

    private void logPose(String key, Pose2d value) {
        if (telemetry != null) telemetry.log(key, value, Pose2d.struct);
        poseLoggingConsumer.accept(new Pair<>(key, value));
    }

    private void logTranslations(String key, Translation2d[] value) {
        if (telemetry != null) telemetry.log(key, value, Translation2d.struct);
        translationListLoggingConsumer.accept(new Pair<>(key, value));
    }

    Follower withTelemetry(org.wpilib.telemetry.TelemetryTable table) {
        requireInactive();
        telemetry = table;
        return this;
    }

    /**
     * Overrides the rotational output of all {@code FollowPath} commands.
     *
     * <p>The supplier is called every execution cycle while active and must return omega in
     * radians per second. This overload bypasses BLine's rotational velocity and acceleration
     * limits so the caller owns the final path-follower omega command.
     *
     * <p>Call {@link #clearRotationOverride()} when the override should stop affecting
     * path following.
     *
     * @param supplier supplies omega in radians per second
     * @throws IllegalArgumentException if supplier is null
     */
    static void overrideRotation(DoubleSupplier supplier) {
        overrideRotation(supplier, RotationOverrideBehavior.BYPASS_CONSTRAINTS);
    }

    /**
     * Overrides the rotational output of all {@code FollowPath} commands.
     *
     * <p>The supplier is called every execution cycle while active and must return omega in
     * radians per second. Use {@link RotationOverrideBehavior#RESPECT_CONSTRAINTS}
     * to keep BLine's normal rotational limits, or
     * {@link RotationOverrideBehavior#BYPASS_CONSTRAINTS} when the caller owns the
     * final rotational command.
     *
     * <p>Call {@link #clearRotationOverride()} when the override should stop affecting
     * path following.
     *
     * @param supplier supplies omega in radians per second
     * @param behavior whether the supplied omega should respect or bypass BLine constraints
     * @throws IllegalArgumentException if supplier or behavior is null
     */
    static void overrideRotation(
        DoubleSupplier supplier,
        RotationOverrideBehavior behavior
    ) {
        if (supplier == null) {
            throw new IllegalArgumentException("Rotation override supplier must not be null");
        }
        if (behavior == null) {
            throw new IllegalArgumentException("Rotation override behavior must not be null");
        }

        rotationOverrideBehavior = behavior;
        rotationOverrideSupplier = supplier;
    }

    /**
     * Clears the active rotation override, restoring normal path rotation control.
     */
    static void clearRotationOverride() {
        rotationOverrideSupplier = null;
        rotationOverrideBehavior = RotationOverrideBehavior.BYPASS_CONSTRAINTS;
    }

    private final TranslationGuidance guidance;
    private final PIDController rotationController;

    private void configureControllers() {
        rotationController.setTolerance(Math.toRadians(endRotationTolerance));
        rotationController.enableContinuousInput(-Math.PI, Math.PI);
    }

    /**
     * Sets the consumer for logging pose data during path following.
     * 
     * <p>The consumer receives pairs of (key, Pose2d) for various internal poses such as
     * closest points on path segments.
     * 
     * @param poseLoggingConsumer The consumer to receive pose logging data, or null to disable
     */
    static void setPoseLoggingConsumer(Consumer<Pair<String, Pose2d>> poseLoggingConsumer) {
        Follower.poseLoggingConsumer = poseLoggingConsumer == null ? value -> {} : poseLoggingConsumer;
    }

    /**
     * Sets the consumer for logging translation arrays during path following.
     * 
     * <p>The consumer receives pairs of (key, Translation2d[]) for data such as path waypoints
     * and robot position history.
     * 
     * @param translationListLoggingConsumer The consumer to receive translation list data, or null to disable
     */
    static void setTranslationListLoggingConsumer(Consumer<Pair<String, Translation2d[]>> translationListLoggingConsumer) {
        Follower.translationListLoggingConsumer = translationListLoggingConsumer == null ? value -> {} : translationListLoggingConsumer;
    }

    /**
     * Sets the consumer for logging boolean values during path following.
     * 
     * <p>The consumer receives pairs of (key, Boolean) for state flags such as completion status.
     * 
     * @param booleanLoggingConsumer The consumer to receive boolean logging data, or null to disable
     */
    static void setBooleanLoggingConsumer(Consumer<Pair<String, Boolean>> booleanLoggingConsumer) {
        Follower.booleanLoggingConsumer = booleanLoggingConsumer == null ? value -> {} : booleanLoggingConsumer;
    }

    /**
     * Sets the consumer for logging numeric values during path following.
     * 
     * <p>The consumer receives pairs of (key, Double) for various metrics such as remaining
     * distance, controller outputs, and target indices.
     * 
     * @param doubleLoggingConsumer The consumer to receive double logging data, or null to disable
     */
    static void setDoubleLoggingConsumer(Consumer<Pair<String, Double>> doubleLoggingConsumer) {
        Follower.doubleLoggingConsumer = doubleLoggingConsumer == null ? value -> {} : doubleLoggingConsumer;
    }

    /**
     * Overrides the time source used to compute loop dt.
     *
     * <p>Passing null restores the default WPILib {@link Timer#getTimestamp()} source.
     * This exists primarily to allow deterministic unit tests without HAL timing dependencies.
     */
    static void setTimestampSupplier(Supplier<Double> supplier) {
        timestampSupplier = supplier == null ? Timer::getTimestamp : supplier;
    }
    
    
    private final Path sourcePath;
    private Path path;
    private boolean initialized;
    private boolean active;
    private boolean resetPose;
    private final Supplier<Pose2d> poseSupplier;
    private final Supplier<ChassisVelocities> robotRelativeSpeedsSupplier;
    private final Consumer<ChassisVelocities> robotRelativeSpeedsConsumer;
    private BooleanSupplier shouldFlipPathSupplier;
    private BooleanSupplier shouldMirrorPathSupplier;
    private final Consumer<Pose2d> poseResetConsumer;
    private Path.DefaultGlobalConstraints executionDefaults;
    private double endTranslationTolerance;
    private double endRotationTolerance;
    
    private int rotationElementIndex = NO_ACTIVE_ROTATION_INDEX;
    private TranslationProgress translationProgress;
    private int eventTriggerElementIndex = 0;

    private HolonomicController holonomicController;
    private double lastTimestamp = 0;
    private Pose2d pathInitStartPose = new Pose2d();
    private RotationProgress rotationProgress;
    private TankController tankController;
    private java.util.OptionalDouble tankFinalHeading = java.util.OptionalDouble.empty();
    private boolean rollingEnd;
    private boolean rollingHandoff;
    private boolean executed;
    private Rotation2d currentRotationTargetRad = new Rotation2d();
    private double currentRotationTargetInitRad = 0;
    private List<Pair<PathElement, PathElementConstraint>> pathElementsWithConstraints = new ArrayList<>();

    private int logCounter = 0;
    private ArrayList<Translation2d> robotTranslations = new ArrayList<>();
    private double cachedRemainingDistance = 0.0;
    private final Set<Integer> firedEventTriggerIndices = new HashSet<>();
    private int firedEventTriggerCount = 0;

    private final DriveType driveType;
    private java.util.Optional<DriveDirection> directionOverride = java.util.Optional.empty();
    private DriveDirection direction = DriveDirection.FORWARD;

    Follower(Path path, FollowerConfig config, BooleanSupplier shouldFlip, PendingEvents events) {
        this.events = events;
        sourcePath = Objects.requireNonNull(path, "path");
        this.path = path;
        driveType = config.driveType();
        poseSupplier = config.pose();
        poseResetConsumer = config.resetPose();
        robotRelativeSpeedsSupplier = config.measuredVelocity();
        robotRelativeSpeedsConsumer = config.output();
        guidance = new TranslationGuidance(config.translation(), config.crossTrack());
        rotationController = config.rotation();
        shouldFlipPathSupplier = shouldFlip;
    }

    void withTankDriveDirection(DriveDirection direction) {
        requireInactive();
        directionOverride = java.util.Optional.of(Objects.requireNonNull(direction, "direction"));
    }

    void withShouldFlip(BooleanSupplier supplier) {
        requireInactive();
        shouldFlipPathSupplier = Objects.requireNonNull(supplier, "shouldFlip");
    }

    void withShouldMirror(BooleanSupplier supplier) {
        requireInactive();
        shouldMirrorPathSupplier = Objects.requireNonNull(supplier, "shouldMirror");
    }

    void withPoseReset() {
        requireInactive();
        resetPose = true;
    }

    private void requireInactive() {
        if (active) throw new IllegalStateException("Cannot reconfigure a running path command");
    }

    void initialize() {
        initialized = false;
        executed = false;
        rollingHandoff = false;
        active = true;
        eventExecution = new PendingEvents.Execution();
        pathElementsWithConstraints = new ArrayList<>();
        cachedRemainingDistance = 0.0;
        if (driveType != DriveType.TANK && directionOverride.orElse(DriveDirection.FORWARD) == DriveDirection.BACKWARD) {
            logger.warning("FollowPath: BACKWARD tank drive direction requires DriveType.TANK");
            return;
        }
        PreparedPath prepared;
        try {
            prepared = PreparedPath.create(sourcePath,
                shouldFlipPathSupplier == null ? java.util.Optional.empty() : java.util.Optional.of(shouldFlipPathSupplier.getAsBoolean()),
                shouldMirrorPathSupplier == null ? java.util.Optional.empty() : java.util.Optional.of(shouldMirrorPathSupplier.getAsBoolean()));
        } catch (IllegalArgumentException error) {
            logger.warning("FollowPath: " + error.getMessage());
            return;
        }
        path = prepared.path();
        direction = driveType == DriveType.TANK
            ? directionOverride.orElse(path.getTankDriveDirection()) : DriveDirection.FORWARD;
        executionDefaults = prepared.defaults();
        endTranslationTolerance = prepared.translationTolerance();
        endRotationTolerance = prepared.rotationToleranceDegrees();
        pathElementsWithConstraints = prepared.elements();
        rollingEnd = prepared.rollingEnd();
        tankFinalHeading = prepared.tankFinalHeading();
        if (resetPose) {
            if (path.hasAuthoredStart()) {
                poseResetConsumer.accept(path.authoredStartPose(poseSupplier.get().getRotation()));
            } else {
                logger.warning("FollowPath: Pose reset skipped because the path has no authored start");
            }
        }

        // Reset traversal state for a fresh command run.
        rotationElementIndex = NO_ACTIVE_ROTATION_INDEX;
        eventTriggerElementIndex = 0;
        firedEventTriggerIndices.clear();
        firedEventTriggerCount = 0;
        lastTimestamp = timestampSupplier.get();
        pathInitStartPose = poseSupplier.get();
        ChassisVelocities initialMeasured = robotRelativeSpeedsSupplier.get();
        if (!finite(pathInitStartPose.getX(), pathInitStartPose.getY(), pathInitStartPose.getRotation().getRadians(),
            initialMeasured.vx, initialMeasured.vy, initialMeasured.omega, lastTimestamp)) {
            failExecution("Non-finite initial pose, measured velocity, or timestamp");
            return;
        }
        holonomicController = driveType == DriveType.TANK ? null : new HolonomicController(initialMeasured, pathInitStartPose.getRotation());
        tankController = driveType == DriveType.TANK ? new TankController(initialMeasured) : null;
        translationProgress = new TranslationProgress(pathElementsWithConstraints, pathInitStartPose);
        rotationProgress = new RotationProgress(pathElementsWithConstraints.stream().map(Pair::getFirst).toList(), pathInitStartPose);
        currentRotationTargetInitRad = pathInitStartPose.getRotation().getRadians();
        rotationController.reset();
        guidance.reset(endTranslationTolerance);
        configureControllers();
        initialized = true;

        ArrayList<Translation2d> pathTranslations = new ArrayList<>();
        robotTranslations.clear();
        logCounter = 0;
        for (int i = 0; i < pathElementsWithConstraints.size(); i++) {
            if (pathElementsWithConstraints.get(i).getFirst() instanceof TranslationTarget) {
                pathTranslations.add(((TranslationTarget) pathElementsWithConstraints.get(i).getFirst()).translation());
            }
        }
        logBoolean("FollowPath/useTRatioBasedTranslationHandoffs", prepared.handoffMode() == HandoffMode.PROGRESS);
        logTranslations("FollowPath/pathTranslations", pathTranslations.toArray(Translation2d[]::new));
    }

    void execute() {
        if (!initialized) {
            stopCommandedMotion();
            return;
        }
        double now = timestampSupplier.get();
        double dt = now - lastTimestamp;
        lastTimestamp = now;
        logDouble("FollowPath/dtSeconds", dt);

        Pose2d currentPose = poseSupplier.get();
        ChassisVelocities measured = robotRelativeSpeedsSupplier.get();
        if (!finite(currentPose.getX(), currentPose.getY(), currentPose.getRotation().getRadians(),
                measured.vx, measured.vy, measured.omega, now, dt)) {
            failExecution("Non-finite pose, measured velocity, or timestamp");
            return;
        }
        if (dt <= 0) return;
        executed = true;

        int previousTranslationIndex = translationProgress.index();
        translationProgress.advance(currentPose);
        boolean translationHandoffOccurred = translationProgress.index() != previousTranslationIndex;
        logBoolean("FollowPath/translationHandoffOccurred", translationHandoffOccurred);
        if (translationHandoffOccurred) {
            logDouble("FollowPath/translationHandoffFromIndex", (double) previousTranslationIndex);
            logDouble("FollowPath/translationHandoffToIndex", (double) translationProgress.index());
        }
        TranslationProgress.Segment currentSegment = translationProgress.segment(currentPose);
        logDouble("FollowPath/currentSegmentLengthMeters", currentSegment.segmentLength());
        logDouble("FollowPath/currentSegmentProgress", currentSegment.segmentProgress());
        logBoolean("FollowPath/currentSegmentDegenerate", currentSegment.isDegenerate());

        // Translation handoff authorizes projection onto the connected next leg. It never
        // replaces the geometric heading progress with the next segment's start heading.
        int lastRotationElementIndex = rotationElementIndex;
        RotationProgress.Sample rotationSample = rotationProgress.update(currentPose.getTranslation(), translationProgress.index());
        rotationElementIndex = rotationSample.activeIndex() >= 0 ? rotationSample.activeIndex() : NO_ACTIVE_ROTATION_INDEX;
        if (lastRotationElementIndex != rotationElementIndex) {
            currentRotationTargetInitRad = currentPose.getRotation().getRadians();
            logDouble("FollowPath/rotationElementIndex", (double) rotationElementIndex);
        }
        logBoolean("FollowPath/rotationHasActiveTarget", rotationSample.activeIndex() >= 0);
        logDouble("FollowPath/rotationPreviousElementIndex", (double) rotationSample.previousIndex());
        logDouble("FollowPath/segmentProgress", rotationSample.intervalProgress());

        // Events retain their translation-segment progress and authored ordering.
        processEventTriggers(currentPose);

        // Combine distance and cross-track feedback before drivetrain-specific control.
        Translation2d targetTranslation = translationProgress.target();
        double remainingDistance = translationProgress.remainingDistance(poseSupplier.get());
        cachedRemainingDistance = remainingDistance;
        boolean finalPositionReached = translationProgress.isLast()
            && remainingDistance <= endTranslationTolerance;
        if (finalPositionReached && rollingEnd) {
            // Preserve the achievable incoming command, including final queued events.
            // Do not invent an instantaneous speed/heading change at the endpoint.
            rollingHandoff = true;
            if (driveType == DriveType.TANK) {
                // A tank retains its body-frame forward speed and turn rate. Reprojecting
                // the previous field vector at a newer heading would introduce lateral motion.
                var command = tankController.commandedVelocity();
                robotRelativeSpeedsConsumer.accept(new ChassisVelocities(command.forward(), 0, command.omega()));
            } else {
                robotRelativeSpeedsConsumer.accept(holonomicController.robotRelativeCommand(currentPose.getRotation()));
            }
            return;
        }
        TranslationTargetConstraint translationConstraint = (TranslationTargetConstraint) pathElementsWithConstraints.get(translationProgress.index()).getSecond();
        var crossTrack = translationProgress.crossTrack(currentPose);
        var requested = guidance.calculate(currentPose, targetTranslation, remainingDistance, crossTrack.errorMeters(),
            translationConstraint, rollingEnd || remainingDistance > endTranslationTolerance);
        double vx = requested.vx(), vy = requested.vy();
        logPose("FollowPath/closestPoint", new Pose2d(crossTrack.closestPoint(), currentPose.getRotation()));
        logDouble("FollowPath/crossTrackError", crossTrack.errorMeters());
        logDouble("FollowPath/rawTranslationControllerOutput", requested.rawSpeed());
        logDouble("FollowPath/clampedTranslationControllerOutput", requested.clampedSpeed());
        logDouble("FollowPath/translationControllerOutput", requested.speed());
        logDouble("FollowPath/minTranslationVelocityMetersPerSec", translationConstraint.minVelocityMetersPerSec());
        logDouble("FollowPath/maxTranslationVelocityMetersPerSec", translationConstraint.maxVelocityMetersPerSec());
        logBoolean("FollowPath/translationMinimumApplied", requested.minimumApplied());
        logDouble("FollowPath/crossTrackControllerOutput", requested.crossTrackOutput());

        // Final settling is deliberately separate from intermediate geometric interpolation.
        if (finalPositionReached && !rollingEnd && driveType != DriveType.TANK) { vx = 0; vy = 0; }
        double targetRotationRad = finalPositionReached
            ? rotationProgress.finalHeadingRadians() : rotationSample.headingRadians();
        int constraintIndex = finalPositionReached ? rotationProgress.finalElementIndex()
            : rotationSample.activeIndex() >= 0 ? rotationSample.activeIndex() : -1;
        RotationTargetConstraint rotationConstraint = constraintIndex >= 0
            ? (RotationTargetConstraint) pathElementsWithConstraints.get(constraintIndex).getSecond()
            : new RotationTargetConstraint(executionDefaults.getMaxVelocityDegPerSec(), executionDefaults.getMaxAccelerationDegPerSec2());
        currentRotationTargetRad = new Rotation2d(finalPositionReached ? rotationProgress.finalHeadingRadians()
            : constraintIndex >= 0 ? ((RotationTarget) pathElementsWithConstraints.get(constraintIndex).getFirst()).rotation().getRadians()
            : targetRotationRad);
        if (finalPositionReached) rotationElementIndex = rotationProgress.finalElementIndex();
        if (constraintIndex >= 0) logPose("FollowPath/rotationTargetPose", rotationProgress.targetPose(constraintIndex));

        TankController.Target tankTarget = null;
        if (driveType == DriveType.TANK) {
            double norm = Math.hypot(vx, vy);
            double scale = norm > translationConstraint.maxVelocityMetersPerSec()
                ? translationConstraint.maxVelocityMetersPerSec() / norm : 1;
            tankTarget = tankController.target(vx * scale, vy * scale, currentPose,
                finalPositionReached, tankFinalHeading, Math.toRadians(endRotationTolerance), direction);
            targetRotationRad = tankTarget.heading();
            currentRotationTargetRad = new Rotation2d(targetRotationRad);
        }
        targetRotationRad = MathUtil.angleModulus(targetRotationRad);
        double rotationPidOutput = tankTarget != null && !tankTarget.steer() ? 0
            : rotationController.calculate(currentPose.getRotation().getRadians(), targetRotationRad);
        double rotationErrorRad = MathUtil.angleModulus(targetRotationRad - currentPose.getRotation().getRadians());
        double rawOmega = rotationPidOutput;
        double maxOmegaRadPerSec = Math.toRadians(rotationConstraint.maxVelocityDegPerSec());
        double minOmegaRadPerSec = Math.toRadians(rotationConstraint.minVelocityDegPerSec());
        double clampedOmega = Math.clamp(rawOmega, -maxOmegaRadPerSec, maxOmegaRadPerSec);
        boolean shouldApplyRotationMinimum =
            Math.abs(rotationErrorRad) > Math.toRadians(endRotationTolerance);
        double omega = TranslationGuidance.minimumMagnitude(
            clampedOmega,
            minOmegaRadPerSec,
            maxOmegaRadPerSec,
            rotationErrorRad,
            shouldApplyRotationMinimum
        );
        boolean rotationMinimumApplied =
            Math.abs(omega) > Math.abs(clampedOmega) + 1e-9;

        if (tankTarget != null && !tankTarget.steer()
            || driveType != DriveType.TANK && finalPositionReached
                && Math.abs(rotationErrorRad) <= Math.toRadians(endRotationTolerance)) {
            omega = 0;
            rotationMinimumApplied = false;
        }

        DoubleSupplier activeRotationOverrideSupplier = rotationOverrideSupplier;
        RotationOverrideBehavior activeRotationOverrideBehavior = rotationOverrideBehavior;
        boolean rotationOverrideActive = activeRotationOverrideSupplier != null;
        boolean rotationOverrideBypassesConstraints =
            rotationOverrideActive &&
            activeRotationOverrideBehavior == RotationOverrideBehavior.BYPASS_CONSTRAINTS;
        double rotationOverrideOmegaRadPerSec = 0.0;
        if (rotationOverrideActive) {
            rotationOverrideOmegaRadPerSec = activeRotationOverrideSupplier.getAsDouble();
            omega = rotationOverrideOmegaRadPerSec;
            rotationMinimumApplied = false;
        }

        // Apply drivetrain limits and emit one robot-relative command.
        if (!finite(vx, vy, omega)) {
            failExecution("Non-finite controller output");
            return;
        }
        ChassisVelocities targetSpeeds;
        if (tankTarget != null) {
            var limited = tankController.limit(tankTarget, omega, new TankRateLimiter.Limits(
                translationConstraint.maxAccelerationMetersPerSec2(),
                Math.toRadians(rotationConstraint.maxAccelerationDegPerSec2()),
                translationConstraint.maxVelocityMetersPerSec(), maxOmegaRadPerSec), dt, direction);
            double outputOmega = rotationOverrideBypassesConstraints ? rotationOverrideOmegaRadPerSec : limited.velocity().omega();
            if (finalPositionReached && !rotationOverrideActive && !tankTarget.steer()) outputOmega = 0;
            tankController.overrideOmega(outputOmega);
            ChassisVelocities robotRelative = new ChassisVelocities(limited.velocity().forward(), 0, outputOmega);
            robotRelativeSpeedsConsumer.accept(robotRelative);
            targetSpeeds = robotRelative.toFieldRelative(currentPose.getRotation());
        } else {
            targetSpeeds = holonomicController.limit(vx, vy, omega, dt, translationConstraint, rotationConstraint,
                finalPositionReached, Math.abs(rotationErrorRad) <= Math.toRadians(endRotationTolerance),
                rotationOverrideActive, rotationOverrideBypassesConstraints);
            robotRelativeSpeedsConsumer.accept(targetSpeeds.toRobotRelative(currentPose.getRotation()));
        }

        if (logCounter++ % 3 == 0) {
            robotTranslations.add(currentPose.getTranslation());

            // Limit memory usage by keeping only the most recent points
            if (robotTranslations.size() > 300) {
                // Remove oldest entries to keep only the last 300 points
                robotTranslations.subList(0, robotTranslations.size() - 250).clear();
            }

            logTranslations("FollowPath/robotTranslations", robotTranslations.toArray(Translation2d[]::new));
        }
        
        logDouble("FollowPath/remainingPathDistanceMeters", cachedRemainingDistance);
        logDouble("FollowPath/translationElementIndex", (double) translationProgress.index());
        logDouble("FollowPath/rotationElementIndex", (double) rotationElementIndex);
        logDouble("FollowPath/targetRotationDeg", Math.toDegrees(targetRotationRad));
        logDouble("FollowPath/rawRotationControllerOutput", rawOmega);
        logDouble("FollowPath/clampedRotationControllerOutput", clampedOmega);
        logDouble("FollowPath/rotationControllerOutput", omega);
        logDouble("FollowPath/rotationPidOutputRadPerSec", rotationPidOutput);
        logBoolean("FollowPath/rotationOverrideActive", rotationOverrideActive);
        logBoolean("FollowPath/rotationOverrideBypassesConstraints", rotationOverrideBypassesConstraints);
        logDouble("FollowPath/rotationOverrideOmegaRadPerSec", rotationOverrideOmegaRadPerSec);
        logDouble("FollowPath/outputOmegaRadPerSec", targetSpeeds.omega);
        logDouble("FollowPath/minRotationVelocityDegPerSec", rotationConstraint.minVelocityDegPerSec());
        logDouble("FollowPath/maxRotationVelocityDegPerSec", rotationConstraint.maxVelocityDegPerSec());
        logBoolean("FollowPath/rotationMinimumApplied", rotationMinimumApplied);
        logDouble("FollowPath/rotationErrorDeg", Math.toDegrees(currentRotationTargetRad.minus(currentPose.getRotation()).getRadians()));
        logDouble("FollowPath/currentRotationTargetInitRad", currentRotationTargetInitRad);
        logDouble("FollowPath/eventTriggerElementIndex", (double) eventTriggerElementIndex);
        logDouble("FollowPath/eventTriggersFiredCount", (double) firedEventTriggerCount);
    }

    /**
     * Forces commanded chassis motion to zero and resets internal speed history.
     *
     * <p>Used on defensive early exits to avoid leaving stale nonzero velocity commands latched.
     */
    private void stopCommandedMotion() {
        ChassisVelocities zeroSpeeds = new ChassisVelocities();
        robotRelativeSpeedsConsumer.accept(zeroSpeeds);
        if (holonomicController != null) holonomicController.stop();
        if (tankController != null) tankController.stop();
    }

    /**
     * Processes event triggers in path order until the next trigger is not yet reached.
     */
    private void processEventTriggers(Pose2d currentPose) {
        processEventTriggers(currentPose, false);
    }

    private void processEventTriggers(Pose2d currentPose, boolean completed) {
        while (eventTriggerElementIndex < pathElementsWithConstraints.size()) {
            PathElement element = pathElementsWithConstraints.get(eventTriggerElementIndex).getFirst();
            if (!(element instanceof EventTrigger)) {
                eventTriggerElementIndex++;
                continue;
            }
            if (firedEventTriggerIndices.contains(eventTriggerElementIndex)) {
                eventTriggerElementIndex++;
                continue;
            }
            if (!completed && !translationProgress.eventReached(eventTriggerElementIndex, currentPose)) {
                break;
            }
            EventTrigger trigger = (EventTrigger) element;
            events.enqueue(eventExecution, trigger.libKey());
            firedEventTriggerIndices.add(eventTriggerElementIndex);
            firedEventTriggerCount++;
            eventTriggerElementIndex++;
        }
    }

    boolean isFinished() {
        if (!initialized) return true;
        if (!executed) return false;
        boolean lastTranslation = translationProgress.isLast();
        boolean atPosition = lastTranslation && translationProgress.remainingDistance(poseSupplier.get()) <= endTranslationTolerance;
        double measuredHeading = poseSupplier.get().getRotation().getRadians();
        double finalHeading = driveType == DriveType.TANK
            ? tankFinalHeading.orElse(measuredHeading) : rotationProgress.finalHeadingRadians();
        boolean atRotation = Math.abs(MathUtil.angleModulus(finalHeading - measuredHeading))
            <= Math.toRadians(endRotationTolerance);
        boolean finished = rollingEnd ? rollingHandoff : atPosition && atRotation;
        logBoolean("FollowPath/finished", finished);
        logBoolean("FollowPath/finishedIsLastRotationElement", rotationSampleAtEnd());
        logBoolean("FollowPath/finishedIsLastTranslationElement", lastTranslation);
        logBoolean("FollowPath/finishedTranslationAtSetpoint", atPosition);
        logBoolean("FollowPath/finishedRotationAtSetpoint", atRotation);
        return finished;
    }

    private boolean rotationSampleAtEnd() {
        return rotationElementIndex == NO_ACTIVE_ROTATION_INDEX || rotationElementIndex == rotationProgress.finalElementIndex();
    }

    void end(boolean interrupted) {
        // Reaching the final tolerance completes the final segment, just as an
        // intermediate handoff completes its segment. Its remaining events must
        // not depend on physically crossing the exact endpoint. Faults and
        // cancellation never dispatch unreached events.
        if (active && !interrupted && initialized && isFinished()) {
            processEventTriggers(poseSupplier.get(), true);
        }
        active = false;
        if (interrupted) events.cancel(eventExecution);
        if (interrupted || !initialized || !rollingHandoff) stopCommandedMotion();
    }

    private void failExecution(String message) {
        logger.warning("FollowPath: " + message);
        initialized = false;
        events.cancel(eventExecution);
        stopCommandedMotion();
    }

    private static boolean finite(double... values) {
        for (double value : values) if (!Double.isFinite(value)) return false;
        return true;
    }

    /**
     * Gets the current rotation element index in the path.
     * 
     * <p>This index represents the current rotation target being tracked, where both
     * waypoint rotations and standalone rotation targets are counted together.
     * 
     * @return The current rotation element index (0-based), or -1 when no active rotation target remains
     */
    int getCurrentRotationElementIndex() {
        return rotationElementIndex;
    }

    /**
     * Gets the current translation element index in the path.
     * 
     * <p>This index represents the current translation target being tracked, where both
     * waypoint translations and standalone translation targets are counted together.
     * 
     * @return The current translation element index (0-based)
     */
    int getCurrentTranslationElementIndex() {
        return translationProgress == null ? 0 : translationProgress.index();
    }

    /**
     * Gets the estimated remaining path distance from the robot's current position.
     *
     * <p>This value is computed live from the command's current traversal cursor and
     * translation targets. It mirrors the distance basis used by the translation controller
     * during execution.
     *
     * <p>Returns {@code 0.0} when the command is not in a valid traversal state
     * (for example, invalid path or uninitialized/invalid translation cursor).
     *
     * @return Remaining path distance in meters
     */
    double getRemainingPathDistanceMeters() {
        if (!initialized ||
            pathElementsWithConstraints.isEmpty() ||
            translationProgress == null) {
            return 0.0;
        }
        return translationProgress.remainingDistance(poseSupplier.get());
    }

}
