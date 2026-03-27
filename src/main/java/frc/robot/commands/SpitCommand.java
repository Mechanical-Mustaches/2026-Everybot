package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class SpitCommand extends Command {
    IntakeSubsystem intakeSubsystem;
    HopperSubsystem hopperSubsystem;

    public SpitCommand(IntakeSubsystem intakeSubsystem, HopperSubsystem hopperSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        this.hopperSubsystem = hopperSubsystem;
    }

    @Override
    public void initialize() {
        intakeSubsystem.reverseIntake();
        intakeSubsystem.indexOut();
        hopperSubsystem.conveyorOut();
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.stopShooter();
        intakeSubsystem.stopIndex();
        hopperSubsystem.conveyorStop();
    }
}
