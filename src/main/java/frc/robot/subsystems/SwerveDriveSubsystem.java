package frc.robot.subsystems;

import java.io.File;
import java.io.IOException;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveDriveSubsystem extends SubsystemBase {
    private final SwerveDrive swerveDrive;
    private final SwerveDrivePoseEstimator poseEstimator;

    public SwerveDriveSubsystem() {
        try {
            var directory = new File(Filesystem.getDeployDirectory(), "swerve");
            var parser = new SwerveParser(directory);

            this.swerveDrive = parser.createSwerveDrive(5.32);
            // parameter is max speed
        } catch (IOException exception) {
            throw new RuntimeException(exception);
        }

        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

        swerveDrive.setMotorIdleMode(true);

        var rotation = new Rotation2d(swerveDrive.getGyroRotation3d().getX(), swerveDrive.getGyroRotation3d().getY());

        this.poseEstimator = new SwerveDrivePoseEstimator(swerveDrive.kinematics,
                rotation,
                swerveDrive.getModulePositions(),
                null);
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    /**
     * Command to drive the robot using translative values and heading as a
     * setpoint.
     *
     * @param translationX Translation in the X direction.
     * @param translationY Translation in the Y direction.
     * @param headingX     Heading X to calculate angle of the joystick.
     * @param headingY     Heading Y to calculate angle of the joystick.
     * @return Drive command.
     */

    /**
     * Command to drive the robot using translative values and heading as angular
     * velocity.
     *
     * @param translationX     Translation in the X direction.
     * @param translationY     Translation in the Y direction.
     * @param angularRotationX Rotation of the robot to set
     * @return Drive command.
     */
    public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY,
            DoubleSupplier angularRotationX) {
        return run(() -> {
            var x = translationX.getAsDouble() * swerveDrive.getMaximumChassisVelocity();
            var y = translationY.getAsDouble() * swerveDrive.getMaximumChassisVelocity();
            var translation = new Translation2d(x, y);
            var rotation = angularRotationX.getAsDouble() * swerveDrive.getMaximumChassisAngularVelocity();

            swerveDrive.drive(translation, rotation, true, false);

            SmartDashboard.putNumber("MaxChassisAngularVelocity", swerveDrive.getMaximumChassisAngularVelocity());
            SmartDashboard.putNumber("MaxChassisVelocity", swerveDrive.getMaximumChassisVelocity());
            SmartDashboard.putNumber("X", x);
            SmartDashboard.putNumber("Y", y);
            SmartDashboard.putNumber("Rotation", rotation);

        });

    }

    @Override
    public void periodic() {
        // First, tell Limelight your robot's current orientation
        double robotYaw = swerveDrive.getGyro().getRotation3d().getY();
        LimelightHelpers.SetRobotOrientation("", robotYaw, 0.0, 0.0, 0.0, 0.0, 0.0);

        // Get the pose estimate
        LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("");

        // Add it to your pose estimator
        poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.5, .5, 9999999));
        poseEstimator.addVisionMeasurement(
                limelightMeasurement.pose,
                limelightMeasurement.timestampSeconds);
    }
}
