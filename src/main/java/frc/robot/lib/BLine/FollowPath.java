package frc.robot.lib.BLine;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.BLine.Path.PathElement;
import frc.robot.lib.BLine.Path.PathElementConstraint;
import frc.robot.lib.BLine.Path.RotationTarget;
import frc.robot.lib.BLine.Path.RotationTargetConstraint;
import frc.robot.lib.BLine.Path.TranslationTarget;
import frc.robot.lib.BLine.Path.TranslationTargetConstraint;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;

/**
 * A WPILib Command that follows a {@link Path} using PID controllers for translation and rotation.
 * 
 * <p>This command drives the robot along a defined path by tracking translation targets sequentially
 * while simultaneously managing rotation targets. The command uses three PID controllers:
 * <ul>
 *   <li><b>Translation Controller:</b> Calculates command speed by minimizing total path distance remaining</li>
 *   <li><b>Rotation Controller:</b> Controls holonomic rotation toward the current rotation target</li>
 *   <li><b>Cross-Track Controller:</b> Minimizes deviation from the line between waypoints</li>
 * </ul>
 * 
 * <p>The path following algorithm works by:
 * <ol>
 *   <li>Calculating command robot speed via a PID controller minimizing total path distance remaining</li>
 *   <li>Determining velocity direction by pointing toward the current translation target</li>
 *   <li>Advancing to the next translation target when within the handoff radius of the current one</li>
 *   <li>Applying cross-track correction to stay on the line between waypoints</li>
 *   <li>Interpolating rotation based on progress between rotation targets</li>
 *   <li>Applying rate limiting via {@link ChassisRateLimiter} to respect constraints</li>
 * </ol>
 * 
 * <h2>Usage</h2>
 * <p>Use the {@link Builder} class to construct FollowPath commands:
 * <pre>{@code
 * FollowPath.Builder pathBuilder = new FollowPath.Builder(
 *     driveSubsystem,
 *     this::getPose,
 *     this::getRobotRelativeSpeeds,
 *     this::driveRobotRelative,
 *     new PIDController(5.0, 0, 0),  // translation
 *     new PIDController(3.0, 0, 0),  // rotation
 *     new PIDController(2.0, 0, 0)   // cross-track
 * ).withDefaultShouldFlip()
 *  .withPoseReset(this::resetPose);
 * 
 * // Then build commands for specific paths:
 * Command followAuto = pathBuilder.build(new Path("myPath"));
 * }</pre>
 * 
 * <h2>Logging</h2>
 * <p>The command supports optional logging via consumer functions. Set up logging callbacks using:
 * <ul>
 *   <li>{@link #setPoseLoggingConsumer(Consumer)} - Log pose data</li>
 *   <li>{@link #setDoubleLoggingConsumer(Consumer)} - Log numeric values</li>
 *   <li>{@link #setBooleanLoggingConsumer(Consumer)} - Log boolean states</li>
 *   <li>{@link #setTranslationListLoggingConsumer(Consumer)} - Log translation arrays</li>
 * </ul>
 * 
 * @see Path
 * @see Builder
 * @see ChassisRateLimiter
 */
public class FollowPath extends Command {
    private static final java.util.logging.Logger logger = java.util.logging.Logger.getLogger(FollowPath.class.getName());
    private static Consumer<Pair<String, Pose2d>> poseLoggingConsumer = value -> {};
    private static Consumer<Pair<String, Translation2d[]>> translationListLoggingConsumer = value -> {};
    private static Consumer<Pair<String, Double>> doubleLoggingConsumer = value -> {};
    private static Consumer<Pair<String, Boolean>> booleanLoggingConsumer = value -> {};

    private static void logDouble(String key, double value) {
        doubleLoggingConsumer.accept(new Pair<>(key, value));
    }

    private static void logBoolean(String key, boolean value) {
        booleanLoggingConsumer.accept(new Pair<>(key, value));
    }

    private static void logPose(String key, Pose2d value) {
        poseLoggingConsumer.accept(new Pair<>(key, value));
    }

    private final PIDController translationController;
    private final PIDController rotationController;
    private final PIDController crossTrackController;

    private static PIDController createPIDControllerCopy(PIDController source) {
        if (source == null) {
            throw new IllegalArgumentException("Cannot create copy of null PIDController");
        }
        return new PIDController(source.getP(), source.getI(), source.getD());
    }

    private void configureControllers() {
        translationController.setTolerance(path.getEndTranslationToleranceMeters());
        rotationController.setTolerance(Math.toRadians(path.getEndRotationToleranceDeg()));
        crossTrackController.setTolerance(path.getEndTranslationToleranceMeters());
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
    public static void setPoseLoggingConsumer(Consumer<Pair<String, Pose2d>> poseLoggingConsumer) {
        if (poseLoggingConsumer == null) { return; }
        FollowPath.poseLoggingConsumer = poseLoggingConsumer;
    }

    /**
     * Sets the consumer for logging translation arrays during path following.
     * 
     * <p>The consumer receives pairs of (key, Translation2d[]) for data such as path waypoints
     * and robot position history.
     * 
     * @param translationListLoggingConsumer The consumer to receive translation list data, or null to disable
     */
    public static void setTranslationListLoggingConsumer(Consumer<Pair<String, Translation2d[]>> translationListLoggingConsumer) {
        if (translationListLoggingConsumer == null) { return; }
        FollowPath.translationListLoggingConsumer = translationListLoggingConsumer;
    }

    /**
     * Sets the consumer for logging boolean values during path following.
     * 
     * <p>The consumer receives pairs of (key, Boolean) for state flags such as completion status.
     * 
     * @param booleanLoggingConsumer The consumer to receive boolean logging data, or null to disable
     */
    public static void setBooleanLoggingConsumer(Consumer<Pair<String, Boolean>> booleanLoggingConsumer) {
        if (booleanLoggingConsumer == null) { return; }
        FollowPath.booleanLoggingConsumer = booleanLoggingConsumer;
    }

    /**
     * Sets the consumer for logging numeric values during path following.
     * 
     * <p>The consumer receives pairs of (key, Double) for various metrics such as remaining
     * distance, controller outputs, and target indices.
     * 
     * @param doubleLoggingConsumer The consumer to receive double logging data, or null to disable
     */
    public static void setDoubleLoggingConsumer(Consumer<Pair<String, Double>> doubleLoggingConsumer) {
        if (doubleLoggingConsumer == null) { return; }
        FollowPath.doubleLoggingConsumer = doubleLoggingConsumer;
    }
    
    
    private final Path path;
    private final Supplier<Pose2d> poseSupplier;
    private final Supplier<ChassisSpeeds> robotRelativeSpeedsSupplier;
    private final Consumer<ChassisSpeeds> robotRelativeSpeedsConsumer;
    private final Supplier<Boolean> shouldFlipPathSupplier;
    private final Consumer<Pose2d> poseResetConsumer;
    
    private int rotationElementIndex = 0;
    private int translationElementIndex = 0;
    private int prevTranslationElementIndex = 0;

    private ChassisSpeeds lastSpeeds = new ChassisSpeeds();
    private double lastTimestamp = 0;
    private Pose2d pathInitStartPose = new Pose2d();
    private double previousRotationElementTargetRad = 0;   
    private int previousRotationElementIndex = 0;
    private Rotation2d currentRotationTargetRad = new Rotation2d();
    private double currentRotationTargetInitRad = 0;
    private List<Pair<PathElement, PathElementConstraint>> pathElementsWithConstraints = new ArrayList<>();

    private int logCounter = 0;
    private ArrayList<Translation2d> robotTranslations = new ArrayList<>();
    private double cachedRemainingDistance = 0.0;

    /**
     * Builder class for constructing {@link FollowPath} commands with a fluent API.
     * 
     * <p>The Builder allows you to configure a path follower once with all the robot-specific
     * parameters, then build multiple commands for different paths. This avoids repeating
     * the same configuration for each path.
     * 
     * <h2>Required Parameters</h2>
     * <p>The constructor requires:
     * <ul>
     *   <li>Drive subsystem - For command requirements</li>
     *   <li>Pose supplier - Returns current robot pose</li>
     *   <li>Robot-relative speeds supplier - Returns current chassis speeds</li>
     *   <li>Robot-relative speeds consumer - Accepts commanded chassis speeds</li>
     *   <li>Three PID controllers for translation, rotation, and cross-track correction</li>
     * </ul>
     * 
     * <h2>Optional Configuration</h2>
     * <ul>
     *   <li>{@link #withShouldFlip(Supplier)} - Custom alliance flip logic</li>
     *   <li>{@link #withDefaultShouldFlip()} - Use DriverStation alliance for flipping</li>
     *   <li>{@link #withPoseReset(Consumer)} - Reset odometry to path start pose</li>
     * </ul>
     * 
     * <h2>Example</h2>
     * <pre>{@code
     * FollowPath.Builder builder = new FollowPath.Builder(
     *     driveSubsystem,
     *     this::getPose,
     *     this::getSpeeds,
     *     this::drive,
     *     translationPID,
     *     rotationPID,
     *     crossTrackPID
     * ).withDefaultShouldFlip();
     * 
     * Command cmd = builder.build(myPath);
     * }</pre>
     */
    public static class Builder {
        private final SubsystemBase driveSubsystem;
        private final Supplier<Pose2d> poseSupplier;
        private final Supplier<ChassisSpeeds> robotRelativeSpeedsSupplier;
        private final Consumer<ChassisSpeeds> robotRelativeSpeedsConsumer;
        private final PIDController translationController;
        private final PIDController rotationController;
        private final PIDController crossTrackController;
        
        private Supplier<Boolean> shouldFlipPathSupplier = () -> false;
        private Consumer<Pose2d> poseResetConsumer = (pose) -> {};
        
        /**
         * Creates a new FollowPath Builder with the required configuration.
         * 
         * @param driveSubsystem The drive subsystem that the command will require
         * @param poseSupplier Supplier that returns the current robot pose in field coordinates
         * @param robotRelativeSpeedsSupplier Supplier that returns current robot-relative chassis speeds
         * @param robotRelativeSpeedsConsumer Consumer that accepts robot-relative chassis speeds to drive
         * @param translationController PID controller for calculating command speed by minimizing path distance remaining
         * @param rotationController PID controller for rotating toward rotation targets
         * @param crossTrackController PID controller for staying on the line between waypoints
         */
        public Builder(
            SubsystemBase driveSubsystem, 
            Supplier<Pose2d> poseSupplier,
            Supplier<ChassisSpeeds> robotRelativeSpeedsSupplier,
            Consumer<ChassisSpeeds> robotRelativeSpeedsConsumer,
            PIDController translationController,
            PIDController rotationController,
            PIDController crossTrackController
        ) {
            this.driveSubsystem = driveSubsystem;
            this.poseSupplier = poseSupplier;
            this.robotRelativeSpeedsSupplier = robotRelativeSpeedsSupplier;
            this.robotRelativeSpeedsConsumer = robotRelativeSpeedsConsumer;
            this.translationController = translationController;
            this.rotationController = rotationController;
            this.crossTrackController = crossTrackController;
        }

        /**
         * Configures a custom supplier to determine whether the path should be flipped.
         * 
         * <p>When the supplier returns true, the path will be flipped to the opposite alliance
         * side using {@link FlippingUtil} during command initialization.
         * 
         * @param shouldFlipPathSupplier Supplier returning true if the path should be flipped
         * @return This builder for chaining
         */
        public Builder withShouldFlip(Supplier<Boolean> shouldFlipPathSupplier) {
            this.shouldFlipPathSupplier = shouldFlipPathSupplier;
            return this;
        }

        /**
         * Configures the builder to use the default alliance-based path flipping.
         * 
         * <p>When enabled, paths will automatically be flipped when the robot is on the
         * red alliance, based on {@link edu.wpi.first.wpilibj.DriverStation#getAlliance()}.
         * 
         * @return This builder for chaining
         */
        public Builder withDefaultShouldFlip() {
            this.shouldFlipPathSupplier = FollowPath::shouldFlipPath;
            return this;
        }

        /**
         * Configures a consumer to reset the robot's pose at the start of path following.
         * 
         * <p>When set, the command will call this consumer with the path's starting pose
         * during initialization. This is useful for resetting odometry when starting autonomous
         * routines or when the robot is placed at a known location.
         * 
         * @param poseResetConsumer Consumer that resets the robot's pose estimate
         * @return This builder for chaining
         */
        public Builder withPoseReset(Consumer<Pose2d> poseResetConsumer) {
            this.poseResetConsumer = poseResetConsumer;
            return this;
        }

        /**
         * Builds a FollowPath command for the specified path.
         * 
         * <p>The built command will use all the configuration from this builder. Each call
         * to build() creates an independent command that can be scheduled.
         * 
         * @param path The path to follow
         * @return A new FollowPath command configured for the given path
         * @throws IllegalArgumentException if any required controllers are null
         */
        public FollowPath build(Path path) {
            return new FollowPath(
                path,
                driveSubsystem,
                poseSupplier,
                robotRelativeSpeedsSupplier,
                robotRelativeSpeedsConsumer,
                shouldFlipPathSupplier,
                poseResetConsumer,
                translationController,
                rotationController,
                crossTrackController
            );
        }
    }

    /**
     * Determines if the path should be flipped based on the current alliance.
     * 
     * @return true if on the red alliance and the path should be flipped, false otherwise
     */
    private static boolean shouldFlipPath() {
        var alliance = edu.wpi.first.wpilibj.DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return alliance.get() == edu.wpi.first.wpilibj.DriverStation.Alliance.Red;
        }
        return false;
    }

    private FollowPath(
        Path path, 
        SubsystemBase driveSubsystem, 
        Supplier<Pose2d> poseSupplier, 
        Supplier<ChassisSpeeds> robotRelativeSpeedsSupplier,
        Consumer<ChassisSpeeds> robotRelativeSpeedsConsumer,
        Supplier<Boolean> shouldFlipPathSupplier,
        Consumer<Pose2d> poseResetConsumer,
        PIDController translationController, 
        PIDController rotationController,
        PIDController crossTrackController
    ) {
        if (translationController == null || rotationController == null || crossTrackController == null) {
            throw new IllegalArgumentException("Controllers must be provided and must not be null");
        }

        this.path = path.copy();
        this.poseSupplier = poseSupplier;
        this.robotRelativeSpeedsSupplier = robotRelativeSpeedsSupplier;
        this.robotRelativeSpeedsConsumer = robotRelativeSpeedsConsumer;
        this.shouldFlipPathSupplier = shouldFlipPathSupplier;
        this.poseResetConsumer = poseResetConsumer;
        this.translationController = translationController;
        this.rotationController = rotationController;
        this.crossTrackController = crossTrackController;
        
        configureControllers();
        
        addRequirements(driveSubsystem);
    }


    @Override
    public void initialize() {
        if (translationController == null || rotationController == null) {
            throw new IllegalArgumentException("Translation and rotation controllers must be provided and must not be null");
        }

        if (!path.isValid()) {
            logger.log(java.util.logging.Level.WARNING, "FollowPath: Path invalid - skipping initialization");
            return;
        }

        if (shouldFlipPathSupplier.get()) {
            path.flip();
        }
        pathElementsWithConstraints = path.getPathElementsWithConstraintsNoWaypoints();

        // find the reset start pose. find the first translation target and use its translation as the start translation and the first rotation target as the start rotation.
        // if no rotation target, use the current robot rotation as the start rotation
        if (pathElementsWithConstraints.isEmpty()) {
            throw new IllegalStateException("Path must contain at least one element");
        }

        Pose2d startPose = path.getStartPose(poseSupplier.get().getRotation());
        poseResetConsumer.accept(startPose);

        rotationElementIndex = 0;
        translationElementIndex = 0;
        prevTranslationElementIndex = 0;
        lastTimestamp = Timer.getTimestamp();
        pathInitStartPose = poseSupplier.get();
        lastSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(robotRelativeSpeedsSupplier.get(), pathInitStartPose.getRotation());
        previousRotationElementTargetRad = pathInitStartPose.getRotation().getRadians();
        previousRotationElementIndex = rotationElementIndex;
        currentRotationTargetInitRad = pathInitStartPose.getRotation().getRadians();
        rotationController.reset();
        translationController.reset();
        configureControllers();

        ArrayList<Translation2d> pathTranslations = new ArrayList<>();
        robotTranslations.clear();
        logCounter = 0;
        for (int i = 0; i < pathElementsWithConstraints.size(); i++) {
            if (pathElementsWithConstraints.get(i).getFirst() instanceof TranslationTarget) {
                pathTranslations.add(((TranslationTarget) pathElementsWithConstraints.get(i).getFirst()).translation());
            }
        }
        translationListLoggingConsumer.accept(new Pair<>("FollowPath/pathTranslations", pathTranslations.toArray(Translation2d[]::new)));
    }

    @Override
    public void execute() {
        if (!path.isValid()) {
            logger.log(java.util.logging.Level.WARNING, "FollowPath: Path invalid - skipping execution");
            return;
        }
        double dt = Timer.getTimestamp() - lastTimestamp;
        lastTimestamp = Timer.getTimestamp();


        Pose2d currentPose = poseSupplier.get();

        // Ensure we have valid indices
        if (translationElementIndex >= pathElementsWithConstraints.size()) {
            logger.warning("FollowPath: Translation element index out of bounds");
            return;
        }
        if (!(pathElementsWithConstraints.get(translationElementIndex).getFirst() instanceof TranslationTarget)) {
            logger.warning("FollowPath: Expected TranslationTarget at index " + translationElementIndex);
            return;
        }

        TranslationTarget currentTranslationTarget = (TranslationTarget) pathElementsWithConstraints.get(translationElementIndex).getFirst();
        // check to see if we are in the intermediate handoff radius of the current target translation
        if (currentPose.getTranslation().getDistance(currentTranslationTarget.translation()) <= 
            currentTranslationTarget.intermediateHandoffRadiusMeters().orElse(path.getDefaultGlobalConstraints().getIntermediateHandoffRadiusMeters())) {
            // if we are in the intermediate handoff radius of the current target translation,
            // switch to the next translation element

            for (int i = translationElementIndex + 1; i < pathElementsWithConstraints.size(); i++) {
                if (pathElementsWithConstraints.get(i).getFirst() instanceof TranslationTarget) {
                    prevTranslationElementIndex = translationElementIndex;
                    translationElementIndex = i;
                    break;
                }
            }
            currentTranslationTarget = (TranslationTarget) pathElementsWithConstraints.get(translationElementIndex).getFirst();
        }

        // Switch rotation targets based on progress along the path
        // Find the next rotation target that we haven't reached yet

        int lastRotationElementIndex = rotationElementIndex;
        while (rotationElementIndex < pathElementsWithConstraints.size()) {
            PathElement currentElement = pathElementsWithConstraints.get(rotationElementIndex).getFirst();

            // Skip non-rotation targets
            if (!(currentElement instanceof RotationTarget)) {
                rotationElementIndex++;
                continue;
            }

            // Check if we should stay at this rotation target or move to the next
            if (isRotationTRatioGreater()) {
                // We haven't reached this target's t_ratio yet, so stay here
                break;
            } else {
                // We've passed this target's t_ratio, log and move to next
                logDouble("FollowPath/rotationElementIndex", (double) rotationElementIndex);
                rotationElementIndex++;

                // If we've reached the end, stop
                if (rotationElementIndex >= pathElementsWithConstraints.size()) {
                    break;
                }
                // Continue searching for the next valid rotation target
            }
        }

        if (lastRotationElementIndex != rotationElementIndex &&
            pathElementsWithConstraints.get(lastRotationElementIndex).getFirst() instanceof RotationTarget) {
            previousRotationElementTargetRad = ((RotationTarget) pathElementsWithConstraints.get(lastRotationElementIndex).getFirst()).rotation().getRadians();
            previousRotationElementIndex = lastRotationElementIndex;
            currentRotationTargetInitRad = currentPose.getRotation().getRadians();
        }

        Translation2d targetTranslation = null;
        if (translationElementIndex < pathElementsWithConstraints.size() &&
            pathElementsWithConstraints.get(translationElementIndex).getFirst() instanceof TranslationTarget) {
            targetTranslation = ((TranslationTarget) pathElementsWithConstraints.get(translationElementIndex).getFirst()).translation();
        } else {
            // Fallback to current pose if no valid translation target
            targetTranslation = currentPose.getTranslation();
        }
        double remainingDistance = calculateRemainingPathDistance();
        double angleToTarget = Math.atan2(
            targetTranslation.getY() - currentPose.getTranslation().getY(),
            targetTranslation.getX() - currentPose.getTranslation().getX()
        );
        double translationControllerOutput = -translationController.calculate(remainingDistance, 0);

        // Cache the remaining distance for logging
        cachedRemainingDistance = remainingDistance;
        double vx = translationControllerOutput * Math.cos(angleToTarget);
        double vy = translationControllerOutput * Math.sin(angleToTarget);

        double crossTrackError = calculateCrossTrackError();
        double crossTrackControllerOutput = -crossTrackController.calculate(crossTrackError, 0);
        vx += crossTrackControllerOutput * Math.cos(angleToTarget - Math.PI / 2);
        vy += crossTrackControllerOutput * Math.sin(angleToTarget - Math.PI / 2);

        double targetRotation;
        RotationTargetConstraint rotationConstraint;

        if (rotationElementIndex < pathElementsWithConstraints.size() &&
            pathElementsWithConstraints.get(rotationElementIndex).getFirst() instanceof RotationTarget) {

            RotationTarget currentRotationTarget = (RotationTarget) pathElementsWithConstraints.get(rotationElementIndex).getFirst();
            if (!(pathElementsWithConstraints.get(rotationElementIndex).getSecond() instanceof RotationTargetConstraint)) {
                logger.warning("FollowPath: Expected RotationTargetConstraint at index " + rotationElementIndex);
                return;
            }
            rotationConstraint = (RotationTargetConstraint) pathElementsWithConstraints.get(rotationElementIndex).getSecond();
            currentRotationTargetRad = currentRotationTarget.rotation();

            if (currentRotationTarget.profiledRotation()) {
                double remainingRotationDistance = calculateRemainingDistanceToRotationTarget();
                double rotationSegmentDistance = calculateRotationTargetSegmentDistance();

                logDouble("FollowPath/remainingRotationDistance", remainingRotationDistance);
                logDouble("FollowPath/rotationSegmentDistance", rotationSegmentDistance);

                // Avoid divide by zero and handle edge cases
                double segmentProgress = 0.0;
                if (rotationSegmentDistance > 1e-6) { // Use small epsilon instead of just > 0
                    segmentProgress = 1 - remainingRotationDistance / rotationSegmentDistance;
                    // Clamp to valid range
                    segmentProgress = Math.max(0.0, Math.min(1.0, segmentProgress));

                    double endTranslationTolerance = path.getEndTranslationToleranceMeters();
                    if (endTranslationTolerance > 0) {
                        // Treat the rotation as complete once we are within the linear tolerance of the target.
                        // This prevents us from undershooting the final rotation when translation stops early.
                        double effectiveTolerance = Math.min(endTranslationTolerance, rotationSegmentDistance);
                        if (remainingRotationDistance <= effectiveTolerance) {
                            segmentProgress = 1.0;
                        }
                    }
                } else if (rotationSegmentDistance < 0) {
                    logger.warning("FollowPath: Negative rotation segment distance: " + rotationSegmentDistance);
                    segmentProgress = 0.0;
                }
                logDouble("FollowPath/segmentProgress", segmentProgress);

                // Calculate the shortest angular path from current robot rotation to target
                double endRotation = currentRotationTarget.rotation().getRadians();
                // Normalize the rotation difference to [-π, π] to take shortest path
                double rotationDifference = MathUtil.angleModulus(endRotation - previousRotationElementTargetRad);

                // Interpolate along the shortest path
                targetRotation = previousRotationElementTargetRad + segmentProgress * rotationDifference;
            } else {
                targetRotation = MathUtil.angleModulus(currentRotationTarget.rotation().getRadians());
            }

        } else {
            targetRotation = previousRotationElementTargetRad;
            currentRotationTargetRad = new Rotation2d(targetRotation);
            rotationConstraint = new RotationTargetConstraint(
                    path.getDefaultGlobalConstraints().getMaxVelocityDegPerSec(), 
                    path.getDefaultGlobalConstraints().getMaxAccelerationDegPerSec2()
                );
        }
        double omega = rotationController.calculate(currentPose.getRotation().getRadians(), targetRotation);

        if (!(pathElementsWithConstraints.get(translationElementIndex).getSecond() instanceof TranslationTargetConstraint)) {
            logger.warning("FollowPath: Expected TranslationTargetConstraint at index " + translationElementIndex);
            return;
        }
        TranslationTargetConstraint translationConstraint = (TranslationTargetConstraint) pathElementsWithConstraints.get(translationElementIndex).getSecond();

        ChassisSpeeds targetSpeeds = new ChassisSpeeds(vx, vy, omega);
        targetSpeeds = ChassisRateLimiter.limit(
            targetSpeeds, 
            lastSpeeds, 
            dt, 
            translationConstraint.maxAccelerationMetersPerSec2(),
            Math.toRadians(rotationConstraint.maxAccelerationDegPerSec2()),
            translationConstraint.maxVelocityMetersPerSec(),
            Math.toRadians(rotationConstraint.maxVelocityDegPerSec())
        );

        robotRelativeSpeedsConsumer.accept(ChassisSpeeds.fromFieldRelativeSpeeds(targetSpeeds, currentPose.getRotation()));

        lastSpeeds = targetSpeeds;

        if (logCounter++ % 3 == 0) {
            robotTranslations.add(currentPose.getTranslation());

            // Limit memory usage by keeping only the most recent points
            if (robotTranslations.size() > 300) {
                // Remove oldest entries to keep only the last 300 points
                robotTranslations.subList(0, robotTranslations.size() - 250).clear();
            }

            translationListLoggingConsumer.accept(new Pair<>("FollowPath/robotTranslations", robotTranslations.toArray(Translation2d[]::new)));
        }
        

        logDouble("FollowPath/calculateRemainingPathDistance", cachedRemainingDistance);
        logDouble("FollowPath/translationElementIndex", (double) translationElementIndex);
        logDouble("FollowPath/rotationElementIndex", (double) rotationElementIndex);
        logDouble("FollowPath/targetRotation", targetRotation);
        logDouble("FollowPath/rotationControllerOutput", omega);

        logDouble("FollowPath/currentRotationTargetInitRad", currentRotationTargetInitRad);

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
     * Calculates the remaining distance to the current rotation target position.
     * 
     * <p>Used for interpolating rotation based on progress between rotation targets.
     * 
     * @return The remaining distance in meters to where the current rotation should be achieved
     */
    private double calculateRemainingDistanceToRotationTarget() {
        Translation2d previousTranslation = poseSupplier.get().getTranslation();
        double remainingDistance = 0;
        if (isRotationNextSegment()) {
            for (int i = translationElementIndex; i <= rotationElementIndex; i++) {
                if (pathElementsWithConstraints.get(i).getFirst() instanceof TranslationTarget) {
                    remainingDistance += previousTranslation.getDistance(
                        ((TranslationTarget) pathElementsWithConstraints.get(i).getFirst()).translation()
                    );
                    previousTranslation = ((TranslationTarget) pathElementsWithConstraints.get(i).getFirst()).translation();
                } else if (pathElementsWithConstraints.get(i).getFirst() instanceof RotationTarget) {
                    remainingDistance += previousTranslation.getDistance(
                        calculateRotationTargetTranslation(i)
                    );
                    previousTranslation = calculateRotationTargetTranslation(i);
                }
            }
        } else {
            remainingDistance = previousTranslation.getDistance(
                calculateRotationTargetTranslation(rotationElementIndex)
            );
        }
        
        return remainingDistance;
    }

    /**
     * Calculates the total distance of the segment between the previous and current rotation targets.
     * 
     * <p>Used for interpolating rotation based on progress between rotation targets.
     * If there is no previous rotation target, returns the distance from the path start.
     * 
     * @return The segment distance in meters
     */
    private double calculateRotationTargetSegmentDistance() {
        // Find the previous rotation target (immediately before current rotation)

        Translation2d startPoint = previousRotationElementIndex == 0
            ? pathInitStartPose.getTranslation()
            : calculateRotationTargetTranslation(previousRotationElementIndex);
        Translation2d endPoint = calculateRotationTargetTranslation(rotationElementIndex);

        double distance = 0.0;
        Translation2d prev = startPoint;
        int startIdx = previousRotationElementIndex == 0 ? 0 : previousRotationElementIndex + 1;
        for (int i = startIdx; i <= rotationElementIndex; i++) {
            if (i == rotationElementIndex) {
                distance += prev.getDistance(endPoint);
                break;
            }
            if (pathElementsWithConstraints.get(i).getFirst() instanceof TranslationTarget) {
                Translation2d next = ((TranslationTarget) pathElementsWithConstraints.get(i).getFirst()).translation();
                distance += prev.getDistance(next);
                prev = next;
            }
        }
        return distance;
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
        Translation2d prevTranslation;
        if (translationElementIndex > 0) {
            prevTranslation = ((TranslationTarget) pathElementsWithConstraints.get(prevTranslationElementIndex).getFirst()).translation();
        }
        else {
            prevTranslation = pathInitStartPose.getTranslation();
        }

        Pose2d currentPose = poseSupplier.get();
        Translation2d robotPosition = currentPose.getTranslation();

        // Vector from previous point to target point
        double dx = targetTranslation.getX() - prevTranslation.getX();
        double dy = targetTranslation.getY() - prevTranslation.getY();

        // Vector from previous point to robot
        double dxRobot = robotPosition.getX() - prevTranslation.getX();
        double dyRobot = robotPosition.getY() - prevTranslation.getY();

        // Length squared of the line segment
        double segmentLengthSquared = dx * dx + dy * dy;

        if (segmentLengthSquared < 1e-6) {
            // Points are essentially the same, return distance to target
            return robotPosition.getDistance(targetTranslation);
        }

        // Project robot position onto the line (dot product)
        double t = (dxRobot * dx + dyRobot * dy) / segmentLengthSquared;

        // Clamp t to [0, 1] to stay within the segment
        t = Math.max(0.0, Math.min(1.0, t));

        // Find the closest point on the line segment
        double closestX = prevTranslation.getX() + t * dx;
        double closestY = prevTranslation.getY() + t * dy;
        Translation2d closestPoint = new Translation2d(closestX, closestY);

        // Calculate signed cross-track error
        // Positive = right of path, Negative = left of path
        double pathVectorX = targetTranslation.getX() - prevTranslation.getX();
        double pathVectorY = targetTranslation.getY() - prevTranslation.getY();
        double robotVectorX = robotPosition.getX() - prevTranslation.getX();
        double robotVectorY = robotPosition.getY() - prevTranslation.getY();

        // Cross product to determine side: positive = left, negative = right
        double crossProduct = pathVectorX * robotVectorY - pathVectorY * robotVectorX;

        // Return signed distance (positive = right of path, negative = left of path)
        double signedError = robotPosition.getDistance(closestPoint);
        if (crossProduct < 0) {
            signedError = -signedError; // Left of path = negative
        }
        // Right of path = positive (crossProduct > 0), so no change needed

        logPose("FollowPath/closestPoint", new Pose2d(closestPoint, currentPose.getRotation()));
        logDouble("FollowPath/crossTrackError", signedError);

        return signedError;
    }

    /**
     * Calculates the field position where a rotation target should be achieved.
     * 
     * <p>Rotation targets are interpolated between translation targets using their t_ratio.
     * 
     * @param index The index of the rotation target in the path elements list
     * @return The translation where this rotation should be achieved
     */
    private Translation2d calculateRotationTargetTranslation(int index) {
        // Validate index
        if (index < 0 || index >= pathElementsWithConstraints.size()) {
            logger.warning("FollowPath: Invalid index for calculateRotationTargetTranslation: " + index);
            return new Translation2d();
        }

        // find the two encompassing translation targets
        Translation2d translationA = null, translationB = null;
        for (int i = index; i >= 0; i--) {
            if (pathElementsWithConstraints.get(i).getFirst() instanceof TranslationTarget) {
                translationA = ((TranslationTarget) pathElementsWithConstraints.get(i).getFirst()).translation();
                break;
            }
        }
        for (int i = index; i < pathElementsWithConstraints.size(); i++) {
            if (pathElementsWithConstraints.get(i).getFirst() instanceof TranslationTarget) {
                translationB = ((TranslationTarget) pathElementsWithConstraints.get(i).getFirst()).translation();
                break;
            }
        }
        if (translationA == null && translationB == null) {
            logger.warning("FollowPath: No translation targets found around rotation target at index " + index);
            return new Translation2d(); // Return default if no translation targets found
        }
        if (translationA == null) {
            return translationB;
        }
        if (translationB == null) {
            return translationA;
        }

        // If the two translation points are at the same location, return that location
        double segmentLength = translationA.getDistance(translationB);
        if (segmentLength == 0) {
            logPose("FollowPath/calculateRotationTargetTranslation", new Pose2d(translationA, new Rotation2d()));
            
            return translationA;
        }

        double angle = Math.atan2(
            translationB.getY() - translationA.getY(),
            translationB.getX() - translationA.getX()
        );

        if (index >= pathElementsWithConstraints.size() ||
            !(pathElementsWithConstraints.get(index).getFirst() instanceof RotationTarget)) {
            logger.warning("FollowPath: Invalid rotation target index: " + index);
            return new Translation2d(); // Return a default value
        }

        double tRatio = ((RotationTarget) pathElementsWithConstraints.get(index).getFirst()).t_ratio();
        // Ensure tRatio is in valid range
        tRatio = Math.max(0.0, Math.min(1.0, tRatio));

        // Calculate the interpolated point along the segment
        double interpolatedDistance = segmentLength * tRatio;
        Translation2d pointOnSegment = new Translation2d(
            translationA.getX() + Math.cos(angle) * interpolatedDistance,
            translationA.getY() + Math.sin(angle) * interpolatedDistance
        );

        logPose("FollowPath/calculateRotationTargetTranslation", new Pose2d(pointOnSegment, new Rotation2d()));
        
        return pointOnSegment;
    }


    /**
     * Checks if the robot hasn't yet reached the current rotation target's t_ratio position.
     * 
     * @return true if the robot should stay at the current rotation target, false if it should advance
     */
    private boolean isRotationTRatioGreater() {
        if (isRotationNextSegment()) { return true; }
        if (isRotationPreviousSegment()) { return false; }
        if (rotationElementIndex >= pathElementsWithConstraints.size() ||
            !(pathElementsWithConstraints.get(rotationElementIndex).getFirst() instanceof RotationTarget)) { return false; }

        Pose2d currentPose = poseSupplier.get();

        // Find the segment that contains this rotation target
        Translation2d translationA = null;
        Translation2d translationB = null;

        // Find the translation target before the rotation target
        for (int i = rotationElementIndex - 1; i >= 0; i--) {
            if (pathElementsWithConstraints.get(i).getFirst() instanceof TranslationTarget) {
                translationA = ((TranslationTarget) pathElementsWithConstraints.get(i).getFirst()).translation();
                break;
            }
        }

        // Find the translation target after the rotation target
        for (int i = rotationElementIndex + 1; i < pathElementsWithConstraints.size(); i++) {
            if (pathElementsWithConstraints.get(i).getFirst() instanceof TranslationTarget) {
                translationB = ((TranslationTarget) pathElementsWithConstraints.get(i).getFirst()).translation();
                break;
            }
        }

        // If we can't find bounding translation targets, default behavior
        if (translationA == null || translationB == null) {
            return true; // Stay at current rotation target
        }

        // TODO: ADD HANDELING FOR VERY SMALL SEGMENTS
        double segmentLength = translationA.getDistance(translationB);
        if (segmentLength < 1e-6) {
            return true; // Avoid division by zero or very small segments
        }

        // Calculate progress along the segment (0 = at translationA, 1 = at translationB)
        double distanceFromA = currentPose.getTranslation().getDistance(translationA);
        double segmentProgress = distanceFromA / segmentLength;

        // Clamp progress to [0, 1]
        segmentProgress = Math.max(0, Math.min(1, segmentProgress));

        double targetTRatio = ((RotationTarget) pathElementsWithConstraints.get(rotationElementIndex).getFirst()).t_ratio();

        // Return true if we haven't reached the target t_ratio yet (should stay at current target)
        boolean shouldStayAtCurrentTarget = segmentProgress < targetTRatio;

        logBoolean("FollowPath/isRotationTRatioGreater", shouldStayAtCurrentTarget);
        logDouble("FollowPath/segmentProgress", segmentProgress);
        logDouble("FollowPath/targetTRatio", targetTRatio);

        return shouldStayAtCurrentTarget;
    }
    
    /**
     * Checks if the current rotation target is on a previous path segment (already passed).
     * 
     * @return true if the rotation target is on a segment before the current translation segment
     */
    private boolean isRotationPreviousSegment() {
        if (rotationElementIndex > translationElementIndex) { return false; }
        
        for (int i = rotationElementIndex; i < translationElementIndex; i++) {
            if (pathElementsWithConstraints.get(i).getFirst() instanceof TranslationTarget) {
                return true;
            }
        }
        return false;
    }

    /**
     * Checks if the current rotation target is on a future path segment.
     * 
     * @return true if the rotation target is on a segment after the current translation segment
     */
    private boolean isRotationNextSegment() {
        return rotationElementIndex > translationElementIndex;
    }

    @Override
    public boolean isFinished() { // TODO add final velocity tolerance
        if (!path.isValid()) {
            logger.log(java.util.logging.Level.WARNING, "FollowPath: Path invalid - finishing early");
            return true;
        }

        // check if this is the last rotation element
        boolean isLastRotationElement = true;
        for (int i = rotationElementIndex+1; i < pathElementsWithConstraints.size(); i++) {
            if (pathElementsWithConstraints.get(i).getFirst() instanceof RotationTarget) {
                isLastRotationElement = false;
                break;
            }
        }
        // check if this is the last translation element (same pattern as rotation)
        boolean isLastTranslationElement = true;
        for (int i = translationElementIndex+1; i < pathElementsWithConstraints.size(); i++) {
            if (pathElementsWithConstraints.get(i).getFirst() instanceof TranslationTarget) {
                isLastTranslationElement = false;
                break;
            }
        }
        boolean finished = 
            isLastRotationElement && isLastTranslationElement && 
            translationController.atSetpoint() && 
            Math.abs(currentRotationTargetRad.minus(poseSupplier.get().getRotation()).getRadians()) < Math.toRadians(path.getEndRotationToleranceDeg());

        logBoolean("FollowPath/finished", finished);
        return finished;
    }


}
