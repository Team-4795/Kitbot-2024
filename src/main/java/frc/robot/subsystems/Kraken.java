package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Kraken extends SubsystemBase {
    private final TalonFX motor = new TalonFX(0);
    public Kraken() {
    }

    public void spinMotor(double speed) {
        motor.set(speed);
    }
}