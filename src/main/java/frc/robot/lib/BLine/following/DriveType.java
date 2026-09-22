package frc.robot.lib.BLine.following;

/** Drivetrain model selected by robot code, independent of editor preview settings. */
public enum DriveType {
    /** Independent field translation and body rotation. */
    SWERVE,
    /** Forward velocity and body rotation, with no sideways velocity. */
    TANK,
    /** Holonomic translation with one shared linear acceleration budget. */
    MECANUM
}
