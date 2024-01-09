package frc.robot.subsystems.Drivetrain;

public final class Constants {
    public static final double kMaxSpeed = 3.0; // meters per second
    public static final double kMaxAngularSpeed = 2 * Math.PI; // one rotation per second

    public static final double kTrackWidth = 0.381 * 2; // meters
    public static final double kWheelRadius = 0.0508; // meters
    public static final int kGearing = 1; //Very wrong lazy guess...
    public static final double kPositionConversionFactor = (2 * Math.PI * kWheelRadius) / kGearing;
    public static final double kVelocityConversionFactor = (2 * Math.PI * kWheelRadius) / (kGearing * 60);
}
