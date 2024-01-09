package frc.robot.subsystems.Drivetrain;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OIConstants;

public class Drivetrain extends SubsystemBase {

    private final CANSparkMax m_leftLeader = new CANSparkMax(1, MotorType.kBrushless);
    private final CANSparkMax m_leftFollower = new CANSparkMax(2, MotorType.kBrushless);
    private final CANSparkMax m_rightLeader = new CANSparkMax(3, MotorType.kBrushless);
    private final CANSparkMax m_rightFollower = new CANSparkMax(4, MotorType.kBrushless);

    private RelativeEncoder leftEncoder;
    private RelativeEncoder rightEncoder;

    private final AnalogGyro m_gyro = new AnalogGyro(0);

    private final PIDController m_leftPIDController = new PIDController(1, 0, 0);
    private final PIDController m_rightPIDController = new PIDController(1, 0, 0);

    private final DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(Constants.kTrackWidth);

    private final DifferentialDriveOdometry m_odometry;

    // Gains are for example purposes only - must be determined for your own robot!
    private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(1, 3);

    /**
     * Constructs a differential drive object. Sets the encoder distance per pulse
     * and resets the
     * gyro.
     */
    public Drivetrain() {
        m_gyro.reset();

        leftEncoder = m_leftLeader.getEncoder();
        rightEncoder = m_rightLeader.getEncoder();

        m_leftLeader.setSmartCurrentLimit(50);
        m_leftFollower.setSmartCurrentLimit(50);
        m_rightLeader.setSmartCurrentLimit(50);
        m_rightFollower.setSmartCurrentLimit(50);

        m_leftFollower.follow(m_leftLeader);
        m_rightFollower.follow(m_rightLeader);

        // We need to invert one side of the drivetrain so that positive voltages
        // result in both sides moving forward. Depending on how your robot's
        // gearbox is constructed, you might have to invert the left side instead.
        m_rightLeader.setInverted(true);

        // Set the distance per pulse for the drive encoders. We can simply use the
        // distance traveled for one rotation of the wheel divided by the encoder
        // resolution.
        leftEncoder.setPositionConversionFactor(Constants.kPositionConversionFactor);
        leftEncoder.setVelocityConversionFactor(Constants.kVelocityConversionFactor);
        rightEncoder.setPositionConversionFactor(Constants.kPositionConversionFactor);
        rightEncoder.setVelocityConversionFactor(Constants.kVelocityConversionFactor);

        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);

        m_odometry = new DifferentialDriveOdometry(
                m_gyro.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition());


        setDefaultCommand(new RunCommand(() -> {
            this.drive(OIConstants.m_driverController.getLeftY(), OIConstants.m_driverController.getRightX());
        }));
    }

    /**
     * Sets the desired wheel speeds.
     *
     * @param speeds The desired wheel speeds.
     */
    public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
        final double leftFeedforward = m_feedforward.calculate(speeds.leftMetersPerSecond);
        final double rightFeedforward = m_feedforward.calculate(speeds.rightMetersPerSecond);
        final double leftOutput = m_leftPIDController.calculate(leftEncoder.getVelocity(), speeds.leftMetersPerSecond);
        final double rightOutput = m_rightPIDController.calculate(rightEncoder.getVelocity(),
                speeds.rightMetersPerSecond);
        m_leftLeader.setVoltage(leftOutput + leftFeedforward);
        m_rightLeader.setVoltage(rightOutput + rightFeedforward);
    }

    /**
     * Drives the robot with the given linear velocity and angular velocity.
     *
     * @param xSpeed Linear velocity in m/s.
     * @param rot    Angular velocity in rad/s.
     */
    public void drive(double xSpeed, double rot) {
        var wheelSpeeds = m_kinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0.0, rot));
        setSpeeds(wheelSpeeds);
    }

    /** Updates the field-relative position. */
    public void updateOdometry() {
        m_odometry.update(
                m_gyro.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition());
    }

    @Override
    public void periodic(){}
}
