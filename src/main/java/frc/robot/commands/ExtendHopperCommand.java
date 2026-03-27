package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HopperSubsystem;

public class ExtendHopperCommand extends Command {
    HopperSubsystem hopperSubsystem;

    public ExtendHopperCommand() {
        this.hopperSubsystem = hopperSubsystem;
    }

    @Override
    public void initialize() {
        hopperSubsystem.unlatch();
    }
}
