package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class SpinUpCommand extends Command {

    IntakeSubsystem intakeSubsystem;
    Boolean isAuto;

    public SpinUpCommand(IntakeSubsystem intakeSubsystem, Boolean isAuto) {
        this.intakeSubsystem = intakeSubsystem;
        this.isAuto = isAuto;
    }

    @Override
    public void initialize() {
        intakeSubsystem.shoot();
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.stopShooter();
    }

    @Override
    public boolean isFinished() {
        if (isAuto && !intakeSubsystem.isFuelDetected()) {
            return true;
        } else
            return false;
    }
}
