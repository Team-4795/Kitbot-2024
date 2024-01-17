package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Shooter.Shooter;

public class AutoCommands {

    private Shooter shooter;

    public AutoCommands(Shooter shooter) {
     this.shooter = shooter;
    }

    public Command OutakeAmp(){
        return Commands.sequence(
            Commands.runOnce(() -> shooter.Outake(0.5)),
            new WaitCommand(0.5),
            Commands.runOnce(shooter::Stop)        
        );
    }

    public Command OutakeSpeaker(){
        return Commands.sequence(
            Commands.runOnce(() -> shooter.Outake(0.75)),
            new WaitCommand(0.5),
            Commands.runOnce(shooter::Stop)        
        );
    }
}