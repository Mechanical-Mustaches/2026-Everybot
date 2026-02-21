package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class SpinUpCommand extends Command {

    IntakeSubsystem intakeSubsystem;

    public SpinUpCommand(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
    }

    @Override
    public void initialize() {
        intakeSubsystem.shoot();
    }
}
