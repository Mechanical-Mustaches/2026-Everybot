package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.SpinUpToShootCommandGroup;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class ShootAllCommandGroup extends ParallelRaceGroup {

    IntakeSubsystem intakeSubsystem;
    HopperSubsystem hopperSubsystem;

    public ShootAllCommandGroup(IntakeSubsystem intakeSubsystem, HopperSubsystem hopperSubsystem) {
        addCommands(
                new SpinUpToShootCommandGroup(intakeSubsystem, hopperSubsystem, true),
                new WaitCommand(5));
    }
}
