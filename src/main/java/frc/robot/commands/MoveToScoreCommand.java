package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class MoveToScoreCommand extends Command{
    SwerveDriveSubsystem swerve;

    public MoveToScoreCommand(SwerveDriveSubsystem swerve){
        this.swerve = swerve;
    }

    @Override
    public void initialize() {
        swerve.driveCommand(()->swerve.getNearestScoringPoint().x, ()->swerve.getNearestScoringPoint().y, ()->swerve.getRotationToPoint(swerve.getHubPoint()));
    }

 

}
