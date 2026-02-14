package frc.robot.commands;

import java.lang.annotation.Target;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberCommand extends Command {

    // TODO imploment/ create command for buttons
    ClimberSubsystem climberSubsystem;
    
   

    public ClimberCommand(ClimberSubsystem climberSubsystem, boolean extend) {
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
