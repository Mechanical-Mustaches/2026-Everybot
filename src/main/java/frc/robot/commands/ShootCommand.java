package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class ShootCommand extends Command {

    IntakeSubsystem intake;
    HopperSubsystem hopper;

    public ShootCommand(IntakeSubsystem intake, HopperSubsystem hopper) {
        this.intake = intake;
        this.hopper = hopper;
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
