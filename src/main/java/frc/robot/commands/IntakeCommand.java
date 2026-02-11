package frc.robot.commands;

import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class IntakeCommand extends Command {
    IntakeSubsystem intake;

    public IntakeCommand(IntakeSubsystem intake) {
        this.intake = intake;
    }

    @Override
    public void initialize() {
        intake.indexIn();
        intake.shoot();

    }

    @Override
    public void end(boolean interrupted) {
        intake.stopIndex();
        intake.stopShooter();
    }

}
