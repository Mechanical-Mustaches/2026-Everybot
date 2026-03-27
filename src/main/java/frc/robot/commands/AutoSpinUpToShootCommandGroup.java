package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class AutoSpinUpToShootCommandGroup extends SequentialCommandGroup {

    IntakeSubsystem intakeSubsystem;
    HopperSubsystem hopperSubsystem;

    public AutoSpinUpToShootCommandGroup(IntakeSubsystem intakeSubsystem, HopperSubsystem hopperSubsystem) {
        addCommands(
                new SpinUpCommand(intakeSubsystem),
                new WaitCommand(1.5),
                new ParallelCommandGroup(
                        new ShootCommand(intakeSubsystem, hopperSubsystem, true),
                        new InstantCommand(() -> hopperSubsystem.conveyorOut())));
    }

}
