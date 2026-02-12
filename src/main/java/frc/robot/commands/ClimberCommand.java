package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberCommand extends Command {

    // TODO imploment/ create command for buttons
    ClimberSubsystem climberSubsystem;
    boolean extend;
    int stage;

    public ClimberCommand(ClimberSubsystem climberSubsystem, boolean extend, int stage) {
        this.extend = extend;
        this.climberSubsystem = climberSubsystem;
        this.stage = stage;
    }

    @Override
    public void initialize() {
        if (extend) {
            climberSubsystem.extend(stage);

        } else {
            climberSubsystem.retract(stage);
        }
    }

    @Override
    public boolean isFinished() { // cheks is the motors are whitin their range and report
        if (this.extend) {
            return climberSubsystem.isExtended(stage);
        }

        return climberSubsystem.isRetracted(stage);

    }

    @Override
    public void end(boolean interrupted) {
        climberSubsystem.stop();
    }
}
