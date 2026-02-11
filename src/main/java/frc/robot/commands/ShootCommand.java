package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class ShootCommand extends Command {

    IntakeSubsystem intake;

    public ShootCommand(IntakeSubsystem intake) {
        this.intake = intake;
    }

    @Override
    public void initialize() {
        intake.indexOut();
        intake.shoot();

    }

    @Override
    public void end(boolean interrupted) {
        intake.stopIndex();
        intake.stopShooter();
    }

}
