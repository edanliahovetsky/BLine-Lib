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
    // Shared epsilon for all segment-length degeneracy checks.
    private static final double SEGMENT_EPSILON = 1e-6;
    // Epsilon for t-ratio comparisons to avoid floating-point edge jitter.
    private static final double T_RATIO_EPSILON = 1e-9;
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

    private final PIDController translationController;
    private final PIDController rotationController;
    private final PIDController crossTrackController;

    private void configureControllers() {
        translationController.setTolerance(endTranslationTolerance);
        rotationController.setTolerance(Math.toRadians(endRotationTolerance));
        crossTrackController.setTolerance(endTranslationTolerance);
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
    private int translationElementIndex = 0;
    private int eventTriggerElementIndex = 0;

    private ChassisVelocities lastSpeeds = new ChassisVelocities();
    private double lastTimestamp = 0;
    private Pose2d pathInitStartPose = new Pose2d();
    private RotationProgress rotationProgress;
    private TankController tankController;
    private java.util.OptionalDouble tankFinalHeading = java.util.OptionalDouble.empty();
    private boolean rollingEnd;
    private boolean rollingHandoff;
    private boolean executed;
    private boolean reportedTankRecovery;
    private Rotation2d currentRotationTargetRad = new Rotation2d();
    private double currentRotationTargetInitRad = 0;
    private List<Pair<PathElement, PathElementConstraint>> pathElementsWithConstraints = new ArrayList<>();

    private int logCounter = 0;
    private ArrayList<Translation2d> robotTranslations = new ArrayList<>();
    private double cachedRemainingDistance = 0.0;
    private final Set<Integer> firedEventTriggerIndices = new HashSet<>();
    private int firedEventTriggerCount = 0;

    // Snapshot of the currently tracked translation segment and robot progress on it.
    private record TranslationSegmentState(
        int startTranslationIndex,
        int endTranslationIndex,
        Translation2d startTranslation,
        Translation2d endTranslation,
        double segmentLength,
        double segmentProgress
    ) {
        /** @return true when this segment is effectively zero-length. */
        private boolean isDegenerate() {
            return segmentLength < SEGMENT_EPSILON;
        }
    }

    private final DriveType driveType;
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
        translationController = config.translation();
        rotationController = config.rotation();
        crossTrackController = config.crossTrack();
        shouldFlipPathSupplier = shouldFlip;
    }

    void withTankDriveDirection(DriveDirection direction) {
        requireInactive();
        this.direction = Objects.requireNonNull(direction, "direction");
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
        reportedTankRecovery = false;
        active = true;
        eventExecution = new PendingEvents.Execution();
        pathElementsWithConstraints = new ArrayList<>();
        cachedRemainingDistance = 0.0;
        if (driveType != DriveType.TANK && direction != DriveDirection.FORWARD) {
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
        translationElementIndex = findNextTranslationTargetIndex(0);
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
        lastSpeeds = initialMeasured.toFieldRelative(pathInitStartPose.getRotation());
        tankController = driveType == DriveType.TANK ? new TankController(initialMeasured) : null;
        rotationProgress = new RotationProgress(pathElementsWithConstraints.stream().map(Pair::getFirst).toList(), pathInitStartPose);
        currentRotationTargetInitRad = pathInitStartPose.getRotation().getRadians();
        rotationController.reset();
        translationController.reset();
        crossTrackController.reset();
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

        // Phase 1: verify translation cursor integrity before doing any control math.
        if (translationElementIndex >= pathElementsWithConstraints.size()) {
            logger.warning("FollowPath: Translation element index out of bounds");
            stopCommandedMotion();
            return;
        }
        if (!isTranslationTargetAt(translationElementIndex)) {
            logger.warning("FollowPath: Expected TranslationTarget at index " + translationElementIndex);
            stopCommandedMotion();
            return;
        }

        // Phase 2: advance translation target(s). This may skip multiple targets in one cycle.
        int previousTranslationIndex = translationElementIndex;
        advanceTranslationTargets(currentPose);
        boolean translationHandoffOccurred = translationElementIndex != previousTranslationIndex;
        logBoolean("FollowPath/translationHandoffOccurred", translationHandoffOccurred);
        if (translationHandoffOccurred) {
            logDouble("FollowPath/translationHandoffFromIndex", (double) previousTranslationIndex);
            logDouble("FollowPath/translationHandoffToIndex", (double) translationElementIndex);
        }
        if (translationElementIndex >= pathElementsWithConstraints.size() || !isTranslationTargetAt(translationElementIndex)) {
            logger.warning("FollowPath: Invalid translation target after handoff at index " + translationElementIndex);
            stopCommandedMotion();
            return;
        }

        TranslationSegmentState currentSegment = getCurrentTranslationSegmentState(currentPose);
        logDouble("FollowPath/currentSegmentLengthMeters", currentSegment.segmentLength());
        logDouble("FollowPath/currentSegmentProgress", currentSegment.segmentProgress());
        logBoolean("FollowPath/currentSegmentDegenerate", currentSegment.isDegenerate());

        // Translation handoff authorizes projection onto the connected next leg. It never
        // replaces the geometric heading progress with the next segment's start heading.
        int lastRotationElementIndex = rotationElementIndex;
        RotationProgress.Sample rotationSample = rotationProgress.update(currentPose.getTranslation(), translationElementIndex);
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

        // Phase 4: compute translational command vector.
        Translation2d targetTranslation = isTranslationTargetAt(translationElementIndex)
            ? ((TranslationTarget) pathElementsWithConstraints.get(translationElementIndex).getFirst()).translation()
            : currentPose.getTranslation();
        double remainingDistance = calculateRemainingPathDistance();
        cachedRemainingDistance = remainingDistance;
        boolean finalPositionReached = findNextTranslationTargetIndex(translationElementIndex + 1) < 0
            && remainingDistance <= endTranslationTolerance;
        if (finalPositionReached && rollingEnd) {
            // Preserve the achievable incoming command, including final queued events.
            // Do not invent an instantaneous speed/heading change at the endpoint.
            rollingHandoff = true;
            robotRelativeSpeedsConsumer.accept(lastSpeeds.toRobotRelative(currentPose.getRotation()));
            return;
        }
        double angleToTarget = Math.atan2(
            targetTranslation.getY() - currentPose.getTranslation().getY(),
            targetTranslation.getX() - currentPose.getTranslation().getX()
        );

        if (!(pathElementsWithConstraints.get(translationElementIndex).getSecond() instanceof TranslationTargetConstraint)) {
            logger.warning("FollowPath: Expected TranslationTargetConstraint at index " + translationElementIndex);
            stopCommandedMotion();
            return;
        }
        TranslationTargetConstraint translationConstraint = (TranslationTargetConstraint) pathElementsWithConstraints.get(translationElementIndex).getSecond();

        // Clamp translation controller output as to not overpower the crossTrackController output during the velo accel limiting phase
        double rawTranslationControllerOutput = -translationController.calculate(remainingDistance, 0);
        double clampedTranslationControllerOutput = Math.clamp(
            rawTranslationControllerOutput,
            -translationConstraint.maxVelocityMetersPerSec(),
            translationConstraint.maxVelocityMetersPerSec()
        );
        boolean shouldApplyTranslationMinimum =
            rollingEnd || remainingDistance > endTranslationTolerance;
        double translationControllerOutput = applyMinimumMagnitude(
            clampedTranslationControllerOutput,
            translationConstraint.minVelocityMetersPerSec(),
            translationConstraint.maxVelocityMetersPerSec(),
            remainingDistance,
            shouldApplyTranslationMinimum
        );
        boolean translationMinimumApplied =
            Math.abs(translationControllerOutput) > Math.abs(clampedTranslationControllerOutput) + 1e-9;
        logDouble("FollowPath/rawTranslationControllerOutput", rawTranslationControllerOutput);
        logDouble("FollowPath/clampedTranslationControllerOutput", clampedTranslationControllerOutput);
        logDouble("FollowPath/translationControllerOutput", translationControllerOutput);
        logDouble("FollowPath/minTranslationVelocityMetersPerSec", translationConstraint.minVelocityMetersPerSec());
        logDouble("FollowPath/maxTranslationVelocityMetersPerSec", translationConstraint.maxVelocityMetersPerSec());
        logBoolean("FollowPath/translationMinimumApplied", translationMinimumApplied);
        
        // Cache the remaining distance for logging
        cachedRemainingDistance = remainingDistance;
        double vx = translationControllerOutput * Math.cos(angleToTarget);
        double vy = translationControllerOutput * Math.sin(angleToTarget);

        double crossTrackError = calculateCrossTrackError();

        // dont clamp cross track controller as users may prefer to tune their controller to be hyper response to cross track
        double crossTrackControllerOutput = -crossTrackController.calculate(crossTrackError, 0);
        logDouble("FollowPath/crossTrackControllerOutput", crossTrackControllerOutput);

        // Rotate the cross-track correction into field frame and add it to translation command.
        vx += crossTrackControllerOutput * Math.cos(angleToTarget - Math.PI / 2);
        vy += crossTrackControllerOutput * Math.sin(angleToTarget - Math.PI / 2);

        // Final settling is deliberately separate from intermediate geometric interpolation.
        if (finalPositionReached && !rollingEnd && driveType != DriveType.TANK) { vx = 0; vy = 0; }
        double targetRotationRad = finalPositionReached
            ? rotationProgress.finalHeadingRadians() : rotationSample.headingRadians();
        int constraintIndex = finalPositionReached ? rotationProgress.finalElementIndex()
            : rotationSample.activeIndex() >= 0 ? rotationSample.activeIndex() : rotationSample.previousIndex();
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
            tankTarget = tankController.target(vx * scale, vy * scale, currentPose, measured,
                finalPositionReached, rollingEnd, tankFinalHeading, Math.toRadians(endRotationTolerance), direction);
            if (tankTarget.phaseChanged()) {
                translationController.reset();
                crossTrackController.reset();
                rotationController.reset();
            }
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
        double omega = applyMinimumMagnitude(
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

        // Phase 6: apply acceleration/velocity limiting and output final command.
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
            if (limited.recovering() && !reportedTankRecovery) {
                logger.warning("FollowPath: Tank motion exceeds a newly applied limit; recovering without a velocity jump");
                reportedTankRecovery = true;
            }
            double outputOmega = rotationOverrideBypassesConstraints ? rotationOverrideOmegaRadPerSec : limited.velocity().omega();
            if (rotationOverrideBypassesConstraints) tankController.overrideOmega(outputOmega);
            ChassisVelocities robotRelative = new ChassisVelocities(limited.velocity().forward(), 0, outputOmega);
            robotRelativeSpeedsConsumer.accept(robotRelative);
            targetSpeeds = robotRelative.toFieldRelative(currentPose.getRotation());
        } else {
            targetSpeeds = ChassisRateLimiter.limit(new ChassisVelocities(vx, vy, omega), lastSpeeds, dt,
                translationConstraint.maxAccelerationMetersPerSec2(),
                Math.toRadians(rotationConstraint.maxAccelerationDegPerSec2()),
                translationConstraint.maxVelocityMetersPerSec(), maxOmegaRadPerSec);
            if (rotationOverrideBypassesConstraints) targetSpeeds.omega = rotationOverrideOmegaRadPerSec;
            robotRelativeSpeedsConsumer.accept(targetSpeeds.toRobotRelative(currentPose.getRotation()));
        }
        lastSpeeds = targetSpeeds;

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
        logDouble("FollowPath/translationElementIndex", (double) translationElementIndex);
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

    private static double applyMinimumMagnitude(
        double value,
        double minimumMagnitude,
        double maximumMagnitude,
        double directionWhenZero,
        boolean enabled
    ) {
        if (!enabled || minimumMagnitude <= 0) {
            return value;
        }

        double boundedMinimum = maximumMagnitude > 0
            ? Math.min(minimumMagnitude, maximumMagnitude)
            : minimumMagnitude;
        if (Math.abs(value) >= boundedMinimum) {
            return value;
        }

        double sign = Math.signum(value);
        if (sign == 0.0) {
            sign = Math.signum(directionWhenZero);
        }
        if (sign == 0.0) {
            sign = 1.0;
        }
        return sign * boundedMinimum;
    }

    /**
     * Forces commanded chassis motion to zero and resets internal speed history.
     *
     * <p>Used on defensive early exits to avoid leaving stale nonzero velocity commands latched.
     */
    private void stopCommandedMotion() {
        ChassisVelocities zeroSpeeds = new ChassisVelocities();
        robotRelativeSpeedsConsumer.accept(zeroSpeeds);
        lastSpeeds = zeroSpeeds;
    }

    private boolean isTranslationTargetAt(int index) {
        return index >= 0 &&
            index < pathElementsWithConstraints.size() &&
            pathElementsWithConstraints.get(index).getFirst() instanceof TranslationTarget;
    }

    private boolean isRotationTargetAt(int index) {
        return index >= 0 &&
            index < pathElementsWithConstraints.size() &&
            pathElementsWithConstraints.get(index).getFirst() instanceof RotationTarget;
    }

    /**
     * Advances translation targets until the current target is no longer handoff-eligible.
     *
     * <p>This intentionally supports "draining" through multiple targets in one cycle,
     * which avoids one-cycle stalls on chains of tiny/degenerate segments.
     */
    private void advanceTranslationTargets(Pose2d currentPose) {
        while (true) {
            if (translationElementIndex >= pathElementsWithConstraints.size() ||
                !(pathElementsWithConstraints.get(translationElementIndex).getFirst() instanceof TranslationTarget)) {
                return;
            }

            int nextTranslationIndex = findNextTranslationTargetIndex(translationElementIndex + 1);
            if (nextTranslationIndex < 0) {
                return;
            }

            TranslationTarget currentTranslationTarget = (TranslationTarget) pathElementsWithConstraints.get(translationElementIndex).getFirst();
            double handoffRadius = currentTranslationTarget.intermediateHandoffRadiusMeters()
                .orElse(executionDefaults.getIntermediateHandoffRadiusMeters());

            TranslationSegmentState currentSegment = getCurrentTranslationSegmentState(currentPose);
            if (!shouldHandoffTranslationTarget(currentPose, currentTranslationTarget, currentSegment, handoffRadius)) {
                return;
            }

            translationElementIndex = nextTranslationIndex;
        }
    }

    /**
     * Determines whether the current translation target should hand off to the next target.
     *
     * <p>Degenerate segments always hand off immediately to avoid zero-length deadlocks.
     */
    private boolean shouldHandoffTranslationTarget(
        Pose2d currentPose,
        TranslationTarget currentTranslationTarget,
        TranslationSegmentState currentSegment,
        double handoffRadius
    ) {
        double distanceToTarget = currentPose.getTranslation().getDistance(currentTranslationTarget.translation());

        if (currentSegment.isDegenerate()) {
            return true;
        }

        if (currentTranslationTarget.handoffMode().orElseThrow() == HandoffMode.RADIUS) {
            return distanceToTarget <= handoffRadius;
        }

        double handoffThreshold = 1.0 - (handoffRadius / currentSegment.segmentLength());
        handoffThreshold = Math.max(0.0, Math.min(1.0, handoffThreshold));
        return currentSegment.segmentProgress() >= handoffThreshold
            || distanceToTarget <= handoffRadius;
    }

    /**
     * Computes segment start/end geometry for the current translation cursor.
     *
     * <p>If the cursor is invalid, returns a degenerate segment at the robot position
     * so callers can handle the error path uniformly.
     */
    private TranslationSegmentState getCurrentTranslationSegmentState(Pose2d currentPose) {
        if (translationElementIndex < 0 || translationElementIndex >= pathElementsWithConstraints.size() ||
            !(pathElementsWithConstraints.get(translationElementIndex).getFirst() instanceof TranslationTarget)) {
            Translation2d currentTranslation = currentPose.getTranslation();
            return new TranslationSegmentState(
                -1,
                translationElementIndex,
                currentTranslation,
                currentTranslation,
                0.0,
                1.0
            );
        }

        int startTranslationIndex = findPreviousTranslationTargetIndex(translationElementIndex - 1);
        Translation2d startTranslation = startTranslationIndex >= 0
            ? getTranslationAtIndex(startTranslationIndex)
            : pathInitStartPose.getTranslation();
        Translation2d endTranslation = getTranslationAtIndex(translationElementIndex);
        double segmentLength = startTranslation.getDistance(endTranslation);
        double segmentProgress = segmentLength < SEGMENT_EPSILON
            ? 1.0
            : calculateSegmentProjectionT(startTranslation, endTranslation, currentPose.getTranslation());

        return new TranslationSegmentState(
            startTranslationIndex,
            translationElementIndex,
            startTranslation,
            endTranslation,
            segmentLength,
            segmentProgress
        );
    }

    /**
     * Finds the next translation target index at or after {@code startIndex}.
     *
     * @return translation index or -1 if none exists
     */
    private int findNextTranslationTargetIndex(int startIndex) {
        for (int i = Math.max(startIndex, 0); i < pathElementsWithConstraints.size(); i++) {
            if (pathElementsWithConstraints.get(i).getFirst() instanceof TranslationTarget) {
                return i;
            }
        }
        return -1;
    }

    /**
     * Finds the previous translation target index at or before {@code startIndex}.
     *
     * @return translation index or -1 if none exists
     */
    private int findPreviousTranslationTargetIndex(int startIndex) {
        for (int i = Math.min(startIndex, pathElementsWithConstraints.size() - 1); i >= 0; i--) {
            if (pathElementsWithConstraints.get(i).getFirst() instanceof TranslationTarget) {
                return i;
            }
        }
        return -1;
    }

    /**
     * Returns translation at a known translation target index, or start pose translation if invalid.
     */
    private Translation2d getTranslationAtIndex(int translationIndex) {
        if (translationIndex >= 0 && translationIndex < pathElementsWithConstraints.size() &&
            pathElementsWithConstraints.get(translationIndex).getFirst() instanceof TranslationTarget) {
            return ((TranslationTarget) pathElementsWithConstraints.get(translationIndex).getFirst()).translation();
        }
        return pathInitStartPose.getTranslation();
    }
    
    /**
     * Calculates the total remaining path distance from the robot's current position.
     * 
     * <p>This is used by the translation controller to calculate command speed.
     * Sums the distances from the current position through all remaining translation targets.
     * 
     * @return The remaining path distance in meters
     */
    private double calculateRemainingPathDistance() {
        Translation2d previousTranslation = poseSupplier.get().getTranslation();
        double remainingDistance = 0;
        for (int i = translationElementIndex; i < pathElementsWithConstraints.size(); i++) {
            if (pathElementsWithConstraints.get(i).getFirst() instanceof TranslationTarget) {
                remainingDistance += previousTranslation.getDistance(
                    ((TranslationTarget) pathElementsWithConstraints.get(i).getFirst()).translation()
                );
                previousTranslation = ((TranslationTarget) pathElementsWithConstraints.get(i).getFirst()).translation();
            }
        }
        return remainingDistance;
    }

    /**
     * Calculates the signed cross-track error from the robot to the line between waypoints.
     * 
     * <p>Positive values indicate the robot is to the right of the path, negative values
     * indicate the robot is to the left of the path.
     * 
     * @return The signed cross-track error in meters
     */
    private double calculateCrossTrackError() {
        Translation2d targetTranslation = ((TranslationTarget) pathElementsWithConstraints.get(translationElementIndex).getFirst()).translation();
        Translation2d prevTranslation = getCurrentTranslationSegmentStart();

        Pose2d currentPose = poseSupplier.get();
        Translation2d robotPosition = currentPose.getTranslation();

        // Find closest point on the segment using shared projection utility
        Translation2d closestPoint = calculateProjectedPointOnSegment(prevTranslation, targetTranslation, robotPosition);

        // Calculate signed cross-track error
        // Positive = left of the directed segment, negative = right
        double pathVectorX = targetTranslation.getX() - prevTranslation.getX();
        double pathVectorY = targetTranslation.getY() - prevTranslation.getY();
        double robotVectorX = robotPosition.getX() - prevTranslation.getX();
        double robotVectorY = robotPosition.getY() - prevTranslation.getY();

        // Cross product to determine side: positive = left, negative = right
        double crossProduct = pathVectorX * robotVectorY - pathVectorY * robotVectorX;

        // Collinear overshoot is longitudinal error, not a sideways correction.
        double signedError = Math.abs(crossProduct) <= SEGMENT_EPSILON * prevTranslation.getDistance(targetTranslation)
            ? 0 : Math.copySign(robotPosition.getDistance(closestPoint), crossProduct);

        logPose("FollowPath/closestPoint", new Pose2d(closestPoint, currentPose.getRotation()));
        logDouble("FollowPath/crossTrackError", signedError);

        return signedError;
    }

    /**
     * Calculates the clamped projection ratio of a point onto a segment.
     *
     * @param segmentStart The start of the segment
     * @param segmentEnd The end of the segment
     * @param point The point to project
     * @return Projection ratio along the segment in [0, 1]
     */
    private double calculateSegmentProjectionT(
        Translation2d segmentStart,
        Translation2d segmentEnd,
        Translation2d point
    ) {
        double dx = segmentEnd.getX() - segmentStart.getX();
        double dy = segmentEnd.getY() - segmentStart.getY();
        double segmentLengthSquared = dx * dx + dy * dy;
        if (segmentLengthSquared < SEGMENT_EPSILON) {
            return 0.0;
        }

        double dxPoint = point.getX() - segmentStart.getX();
        double dyPoint = point.getY() - segmentStart.getY();
        double t = (dxPoint * dx + dyPoint * dy) / segmentLengthSquared;
        return Math.max(0.0, Math.min(1.0, t));
    }

    /**
     * Calculates the projected point on a segment for a given position.
     *
     * @param segmentStart The start of the segment
     * @param segmentEnd The end of the segment
     * @param point The point to project
     * @return The projected point on the segment
     */
    private Translation2d calculateProjectedPointOnSegment(
        Translation2d segmentStart,
        Translation2d segmentEnd,
        Translation2d point
    ) {
        double t = calculateSegmentProjectionT(segmentStart, segmentEnd, point);
        double dx = segmentEnd.getX() - segmentStart.getX();
        double dy = segmentEnd.getY() - segmentStart.getY();
        return new Translation2d(
            segmentStart.getX() + t * dx,
            segmentStart.getY() + t * dy
        );
    }

    /**
     * Gets the start point for the current translation segment.
     *
     * <p>This walks backward from the current translation element to find the
     * previous translation target. If none exists, it falls back to the path
     * initialization pose. This keeps cross-track calculations stable when
     * translation targets switch.
     *
     * @return The start translation for the current segment
     */
    private Translation2d getCurrentTranslationSegmentStart() {
        int previousTranslationIndex = findPreviousTranslationTargetIndex(translationElementIndex - 1);
        return previousTranslationIndex >= 0
            ? getTranslationAtIndex(previousTranslationIndex)
            : pathInitStartPose.getTranslation();
    }

    /**
     * Processes event triggers in path order until the next trigger is not yet reached.
     */
    private void processEventTriggers(Pose2d currentPose) {
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
            if (!isEventTriggerTRatioReached(eventTriggerElementIndex, currentPose)) {
                break;
            }
            EventTrigger trigger = (EventTrigger) element;
            events.enqueue(eventExecution, trigger.libKey());
            firedEventTriggerIndices.add(eventTriggerElementIndex);
            firedEventTriggerCount++;
            eventTriggerElementIndex++;
        }
    }

    /**
     * Returns true when the trigger's t_ratio has been reached on its owning segment.
     *
     * <p>Degenerate event segments are treated as immediately reached.
     */
    private boolean isEventTriggerTRatioReached(int eventIndex, Pose2d currentPose) {
        if (eventIndex >= pathElementsWithConstraints.size() ||
            !(pathElementsWithConstraints.get(eventIndex).getFirst() instanceof EventTrigger)) {
            return false;
        }
        if (isEventTriggerNextSegment(eventIndex)) { return false; }
        if (isEventTriggerPreviousSegment(eventIndex)) { return true; }

        Translation2d translationA = pathInitStartPose.getTranslation();
        Translation2d translationB = null;
        for (int i = eventIndex - 1; i >= 0; i--) {
            if (pathElementsWithConstraints.get(i).getFirst() instanceof TranslationTarget) {
                translationA = ((TranslationTarget) pathElementsWithConstraints.get(i).getFirst()).translation();
                break;
            }
        }
        for (int i = eventIndex + 1; i < pathElementsWithConstraints.size(); i++) {
            if (pathElementsWithConstraints.get(i).getFirst() instanceof TranslationTarget) {
                translationB = ((TranslationTarget) pathElementsWithConstraints.get(i).getFirst()).translation();
                break;
            }
        }
        if (translationA == null || translationB == null) {
            logger.warning("FollowPath: Missing translation bounds for event trigger at index " + eventIndex);
            return false;
        }

        double segmentLength = translationA.getDistance(translationB);
        if (segmentLength < SEGMENT_EPSILON) {
            return true;
        }

        double segmentProgress = calculateSegmentProjectionT(
            translationA,
            translationB,
            currentPose.getTranslation()
        );

        double targetTRatio = ((EventTrigger) pathElementsWithConstraints.get(eventIndex).getFirst()).t_ratio();
        return segmentProgress >= targetTRatio;
    }

    private boolean isEventTriggerPreviousSegment(int eventIndex) {
        if (eventIndex > translationElementIndex) { return false; }
        for (int i = eventIndex; i < translationElementIndex; i++) {
            if (pathElementsWithConstraints.get(i).getFirst() instanceof TranslationTarget) {
                return true;
            }
        }
        return false;
    }

    private boolean isEventTriggerNextSegment(int eventIndex) {
        return eventIndex > translationElementIndex;
    }

    boolean isFinished() {
        if (!initialized) return true;
        if (!executed) return false;
        boolean lastTranslation = findNextTranslationTargetIndex(translationElementIndex + 1) < 0;
        boolean atPosition = lastTranslation && calculateRemainingPathDistance() <= endTranslationTolerance;
        boolean atRotation = Math.abs(currentRotationTargetRad.minus(poseSupplier.get().getRotation()).getRadians())
            <= Math.toRadians(endRotationTolerance);
        ChassisVelocities measured = robotRelativeSpeedsSupplier.get();
        boolean stopped = Math.hypot(lastSpeeds.vx, lastSpeeds.vy) < 1e-8 && Math.abs(lastSpeeds.omega) < 1e-8
            && Math.hypot(measured.vx, measured.vy) <= TankController.STOPPED_VELOCITY
            && Math.abs(measured.omega) <= TankController.STOPPED_VELOCITY;
        boolean finished = rollingEnd ? rollingHandoff : atPosition
            && (driveType == DriveType.TANK ? tankController.finished() : atRotation && stopped);
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
        return translationElementIndex;
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
            !isTranslationTargetAt(translationElementIndex)) {
            return 0.0;
        }
        return calculateRemainingPathDistance();
    }

}
