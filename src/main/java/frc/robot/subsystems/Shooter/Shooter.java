package frc.robot.subsystems.Shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    CANSparkMax Shootermotor = new CANSparkMax(10, MotorType.kBrushless);
    CANSparkMax Shootermotor2 = new CANSparkMax(11, MotorType.kBrushless);
    public Shooter() {
        Shootermotor.setSmartCurrentLimit(30);
        Shootermotor2.setSmartCurrentLimit(30);
        Shootermotor2.follow(Shootermotor);
        Shootermotor.burnFlash();
        Shootermotor2.burnFlash();
    }

    public void Outake(double speed) {
        Shootermotor.set(speed);
    }

    public void Intake() {
        Shootermotor.set(Constants.IntakeSpeed1);
    }

    public void Stop() {
        Shootermotor.set(0);
    }
}


