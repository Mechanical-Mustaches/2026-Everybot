package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberCommand extends Command {

    // TODO imploment/ create command for buttons
    ClimberSubsystem climberSubsystem;

    public ClimberCommand(ClimberSubsystem climberSubsystem) {
        this.climberSubsystem = climberSubsystem;
    }

    @Override
    public void initialize() {
        climberSubsystem.climb();
        climberSubsystem.unClimb();
    }

    @Override
    public void end(boolean interrupted){
        climberSubsystem.stop();
    }
}
