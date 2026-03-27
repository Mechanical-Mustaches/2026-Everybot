package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class ShootCommand extends Command {

    IntakeSubsystem intake;
    HopperSubsystem hopper;
    boolean isAuto;

    public ShootCommand(IntakeSubsystem intake, HopperSubsystem hopper, boolean isAuto) {
        this.intake = intake;
        this.hopper = hopper;
        this.isAuto = isAuto;
    }

    public ShootCommand(IntakeSubsystem intake, HopperSubsystem hopper) {
        this.intake = intake;
        this.hopper = hopper;
        this.isAuto = false;
    }

    @Override
    public void initialize() {
        intake.indexOut();
        intake.velocityShoot();
    }

    @Override
    public void execute() {
        intake.velocityShoot();
    }

    @Override
    public void end(boolean interrupted) {
        if (!isAuto) {
            intake.stopIndex();
            intake.stopShooter();
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
