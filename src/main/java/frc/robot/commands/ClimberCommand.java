package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ClimberSubsystem.Stage;

public class ClimberCommand extends Command {

    // TODO imploment/ create command for buttons
    ClimberSubsystem climberSubsystem;

    public ClimberCommand(ClimberSubsystem climberSubsystem) {
        this.climberSubsystem = climberSubsystem;
    }

    @Override
    public void initialize() {
        climberSubsystem.climb();
    }

    @Override
    public boolean isFinished() {
        return climberSubsystem.isDone(Stage.S1);
    }

    @Override
    public void end(boolean interrupted){
        climberSubsystem.stop();
    }
}
