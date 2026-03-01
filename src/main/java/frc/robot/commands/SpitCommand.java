package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class SpitCommand extends Command {
    IntakeSubsystem intakeSubsystem;

    public SpitCommand(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
    }

    @Override
    public void initialize() {
        intakeSubsystem.reverseIntake();
        intakeSubsystem.indexOut();
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.stopShooter();
        intakeSubsystem.stopIndex();
    }
}
