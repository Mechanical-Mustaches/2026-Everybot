package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindThenFollowPath;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class MoveToScoreCommand extends Command {
    SwerveDriveSubsystem swerve;
    PathConstraints constraints;
    Pose2d targetPose;

    public MoveToScoreCommand(SwerveDriveSubsystem swerve) {
        this.swerve = swerve;
        constraints = new PathConstraints(
                swerve.getMaximumChassisVelocity(), 4.0,
                swerve.getMaximumChassisAngularVelocity(), Units.degreesToRadians(720));

        targetPose = new Pose2d(swerve.getNearestScoringPoint().x, swerve.getNearestScoringPoint().y,
                new Rotation2d(swerve.getRotationToPoint(SwerveDriveSubsystem.getHubPoint())));

    }

    @Override
    public void initialize() {
        AutoBuilder.pathfindToPose(targetPose, constraints);
    }

}
