package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class SpinUpToShootCommandGroup extends SequentialCommandGroup {

    IntakeSubsystem intakeSubsystem;
    HopperSubsystem hopperSubsystem;

    public SpinUpToShootCommandGroup(IntakeSubsystem intakeSubsystem, HopperSubsystem hopperSubsystem) {
        addCommands(new SpinUpCommand(intakeSubsystem), new WaitCommand(1.5),
                new ShootCommand(intakeSubsystem, hopperSubsystem));
    }

}
