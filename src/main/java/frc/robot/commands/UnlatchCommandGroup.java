package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.HopperSubsystem;

public class UnlatchCommandGroup extends Command {

    public UnlatchCommandGroup(HopperSubsystem hopperSubsystem) {
        new SequentialCommandGroup(
                new InstantCommand(() -> hopperSubsystem.unlatch()),
                new WaitCommand(0.5),
                new InstantCommand(() -> hopperSubsystem.stopServos()));
    }
}
