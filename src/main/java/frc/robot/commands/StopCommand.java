package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class StopCommand extends Command {

    IntakeSubsystem intakeSubsystem;
    HopperSubsystem hopperSubsystem;

    public StopCommand(IntakeSubsystem intakeSubsystem, HopperSubsystem hopperSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        this.hopperSubsystem = hopperSubsystem;
    }

    @Override
    public void initialize() {
        intakeSubsystem.stopIndex();
        intakeSubsystem.stopShooter();
        hopperSubsystem.conveyorStop();
    }

}
