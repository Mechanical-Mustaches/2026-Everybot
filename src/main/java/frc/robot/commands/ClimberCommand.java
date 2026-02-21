package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ClimberSubsystem.Stage;

public class ClimberCommand extends Command {

    private final ClimberSubsystem climber;
    private final Stage stage;

    public ClimberCommand(ClimberSubsystem climber, Stage stage) {
        this.climber = climber;
        this.stage = stage;
    }

    @Override
    public void initialize() {
        if (!climber.isDone(stage)){
            if (stage == Stage.S1 || stage == Stage.S2 ){
                climber.reverseClimb();
            } else {
                climber.climb();
            }
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
