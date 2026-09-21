package frc.robot.lib.BLine;

/** How an intermediate translation target hands control to the next target. */
public enum HandoffMode {
    /** Switch when the robot is within the handoff distance of the target. */
    RADIUS,
    /** Switch at projected segment progress, retaining the within-radius fallback. */
    PROGRESS
}
