package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class SpinUpToShootCommandGroup extends SequentialCommandGroup {

    IntakeSubsystem intakeSubsystem;
    HopperSubsystem hopperSubsystem;
    boolean isAuto;

    public SpinUpToShootCommandGroup(IntakeSubsystem intakeSubsystem, HopperSubsystem hopperSubsystem, Boolean isAuto) {
        addCommands(new ParallelRaceGroup(
                new SpinUpCommand(intakeSubsystem, isAuto),
                new WaitCommand(1.5)),
                new ParallelCommandGroup(
                        new ShootCommand(intakeSubsystem, hopperSubsystem),
                        new InstantCommand(() -> hopperSubsystem.conveyorOut())));

    }

}
