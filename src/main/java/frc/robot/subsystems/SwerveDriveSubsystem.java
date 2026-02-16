package frc.robot.subsystems;

import java.io.File;
import java.io.IOException;
import java.lang.reflect.Array;
import java.util.function.DoubleSupplier;

import javax.sound.sampled.Line;

import org.dyn4j.geometry.Circle;
import org.opencv.core.Point;
import java.awt.geom.*;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveDriveSubsystem extends SubsystemBase {
    private final SwerveDrive swerveDrive;
    private final SwerveDrivePoseEstimator poseEstimator;
    private final Field2d m_field;
    private static Point kRedHubPoint = new Point(4.034663,4.625594);
    private static Point kBlueHubPoint = new Point(4.034663,4.625594);
    private static double kScoringRadius = 2;
    public static Point getHubPoint(){
        if(DriverStation.getAlliance().get() == DriverStation.Alliance.Blue){
            return kBlueHubPoint;
        } else {
            return kRedHubPoint;
        }
    }
    

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
                new Pose2d());

        this.m_field = new Field2d();

        SmartDashboard.putData("swerveField", m_field);
    }

    public Point getNearestScoringPoint(){
        Point robotPoint = new Point(getPose().getX(),getPose().getY());
        double slope = ((robotPoint.y-getHubPoint().y)/(robotPoint.x-getHubPoint().x));
        
        
        double horizontalDistance = kScoringRadius/Math.sqrt((slope*slope)+1);
        double verticalDistance = Math.sqrt((slope*slope)/(horizontalDistance*horizontalDistance));

        Point scoringPoint = new Point(getHubPoint().x-horizontalDistance, getHubPoint().y-verticalDistance);

        return scoringPoint;

    }

    public double getRotationToPoint(Point point){
        return Math.atan((point.y-getPose().getY())/(point.x-getPose().getX()));
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public void resetGyro(){
        swerveDrive.setGyro(new Rotation3d(0,0,0));
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

        LimelightHelpers.SetRobotOrientation("limelight-front", robotYaw, 0.0, 0.0, 0.0, 0.0, 0.0);
        LimelightHelpers.SetRobotOrientation("limelight-back", robotYaw, 0.0, 0.0, 0.0, 0.0, 0.0);

        // Get the pose estimate
        LimelightHelpers.PoseEstimate frontLimelightMeasurement = LimelightHelpers
                .getBotPoseEstimate_wpiBlue_MegaTag2("limelight-front");
        LimelightHelpers.PoseEstimate backLimelightMeasurement = LimelightHelpers
                .getBotPoseEstimate_wpiBlue_MegaTag2("limelight-back");

        // Add it to your pose estimator
        poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.5, .5, 9999999));

        if (frontLimelightMeasurement != null) {
            poseEstimator.addVisionMeasurement(
                    frontLimelightMeasurement.pose,
                    frontLimelightMeasurement.timestampSeconds);
            double[] frontLimelightEstimatedPose = { frontLimelightMeasurement.pose.getX(),
                    frontLimelightMeasurement.pose.getY() };
            SmartDashboard.putNumberArray("frontLimelightEstimatedPose", frontLimelightEstimatedPose);
            SmartDashboard.putNumber("frontLimelightAveragedist",
                    LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-front").avgTagDist);
        }
        if (backLimelightMeasurement != null) {
            poseEstimator.addVisionMeasurement(
                    backLimelightMeasurement.pose,
                    backLimelightMeasurement.timestampSeconds);
            double[] backLimelightEstimatedPose = { backLimelightMeasurement.pose.getX(),
                    backLimelightMeasurement.pose.getY() };
            SmartDashboard.putNumberArray("backLimelightEstimatedPose", backLimelightEstimatedPose);
            SmartDashboard.putNumber("backLimelightAveragedist",
                    LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-back").avgTagDist);
        }
        poseEstimator.update(swerveDrive.getGyroRotation3d().toRotation2d(), swerveDrive.getModulePositions());
        m_field.setRobotPose(getPose());

    }
}
