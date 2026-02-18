package frc.robot.commands;

import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class IntakeCommand extends Command {
    IntakeSubsystem intake;
    HopperSubsystem hopper;

    public IntakeCommand(IntakeSubsystem intake, HopperSubsystem hopperSubsystem) {
        this.intake = intake;
        this.hopper = hopper;
    }

    @Override
    public void initialize() {
        intake.indexIn();
        intake.intake();
        hopper.conveyorIn();

    }

    @Override
    public void end(boolean interrupted) {
        intake.stopIndex();
        intake.stopShooter();
        hopper.conveyorStop();
    }

}
