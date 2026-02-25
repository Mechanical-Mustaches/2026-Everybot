package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class ConveyorOutCommand extends Command {

    IntakeSubsystem intakeSubsystem;
    HopperSubsystem hopperSubsystem;

    public ConveyorOutCommand(IntakeSubsystem intakeSubsystem, HopperSubsystem hopperSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        this.hopperSubsystem = hopperSubsystem;
    }

    @Override
    public void execute() {
        if (!intakeSubsystem.isAtSpeed()) {
            hopperSubsystem.conveyorIn();

        }

        hopperSubsystem.conveyorOut();
    }

    @Override
    public void end(boolean interrupted) {
        hopperSubsystem.conveyorStop();
    }

}
