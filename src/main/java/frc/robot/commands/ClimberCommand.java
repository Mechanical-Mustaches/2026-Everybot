package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ClimberSubsystem.Stage;

public class ClimberCommand extends Command {

    // TODO imploment/ create command for buttons
    private final ClimberSubsystem climber;
    private final Stage stage;

    public ClimberCommand(ClimberSubsystem climber, Stage stage) {
        this.climber = climber;
        this.stage = stage;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double curPos = climber.getEncoderPosition();
        double target = stage.encoderValue;
        if (curPos < target) {
            climber.dumbClimb();
        } else if (curPos > target) {
            climber.dumbUnClimb();
        }
    }

    @Override
    public boolean isFinished() {
        return climber.isDone(stage);
    }

    @Override
    public void end(boolean interrupted) {
        climber.stop();
    }
}
