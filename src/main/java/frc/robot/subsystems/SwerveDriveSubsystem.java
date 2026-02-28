package frc.robot.subsystems;

import java.io.File;
import java.io.IOException;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import javax.sound.sampled.Line;

import org.dyn4j.geometry.Circle;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import java.awt.geom.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveDriveSubsystem extends SubsystemBase {
    private final SwerveDrive swerveDrive;
    private final Field2d m_field;

    private static Point kRedHubPoint = new Point(4.034663, 4.625594);
    private static Point kBlueHubPoint = new Point(4.034663, 4.625594);
    private static double kScoringRadius = Units.inchesToMeters(60 + 16.5);
    private static double kPositionTolerance = Units.inchesToMeters(3);
    private static double kApproachingTolerance = Units.inchesToMeters(10);

    public static Point getHubPoint() {

        if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
            return kBlueHubPoint;
        } else {
            return kRedHubPoint;
        }
        // return kBlueHubPoint;

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

        RobotConfig config;
        try {
            config = RobotConfig.fromGUISettings();

            // Configure AutoBuilder last
            AutoBuilder.configure(
                    this::getPose, // Robot pose supplier
                    this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
                    this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                    (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given
                                                                          // ROBOT RELATIVE ChassisSpeeds. Also
                                                                          // optionally outputs individual module
                                                                          // feedforwards
                    new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller
                                                    // for holonomic drive trains
                            new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                            new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
                    ),
                    config, // The robot configuration
                    () -> {
                        // Boolean supplier that controls when the path will be mirrored for the red
                        // alliance
                        // This will flip the path being followed to the red side of the field.
                        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                        var alliance = DriverStation.getAlliance();
                        if (alliance.isPresent()) {
                            return alliance.get() == DriverStation.Alliance.Red;
                        }
                        return false;
                    },
                    this // Reference to this subsystem to set requirements

            );
        } catch (Exception e) {
            // Handle exception as needed
            e.printStackTrace();
        }

        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

        swerveDrive.setMotorIdleMode(true);

        this.m_field = new Field2d();

        swerveDrive.setHeadingCorrection(false);

        SmartDashboard.putData("swerveField", m_field);

    }

    public Point getNearestScoringPoint() {

        var robotRotation = getRotationToPoint(getHubPoint());
        var y = kScoringRadius * Math.cos(robotRotation);
        var x = kScoringRadius * Math.sin(robotRotation);

        Point scoringPoint = new Point(getHubPoint().x + x, getHubPoint().y + y);

        return scoringPoint;

    }

    public double distanceToPoint(Point targetPoint) {
        var dX = targetPoint.x - getPose().getX();
        var dY = targetPoint.y - getPose().getY();
        var dist = Math.sqrt((dX * dX) + (dY * dY));
        return dist;
    }

    public boolean isInRange() {
        if (distanceToPoint(getNearestScoringPoint()) <= kPositionTolerance) {
            return true;
        } else
            return false;
    }

    public boolean isApproaching() {
        if (distanceToPoint(getNearestScoringPoint()) <= kApproachingTolerance) {
            return true;
        } else
            return false;
    }

    public void moveToPoint(Point targetPoint, boolean rotateToPoint) {
        swerveDrive.drive(new ChassisSpeeds(
                swerveDrive.getMaximumChassisVelocity() * 0.65 * Math.sin(getRotationToPoint(targetPoint)),
                swerveDrive.getMaximumChassisVelocity() * 0.65 * Math.cos(getRotationToPoint(targetPoint)), 0));
    }

    public double getRotationToPoint(Point point) {
        Pose2d robotPose = getPose();

        double dx = point.x - robotPose.getX();
        double dy = point.y - robotPose.getY();

        Rotation2d angle = new Rotation2d(dx, dy);

        return -(MathUtil.angleModulus(swerveDrive.getGyro().getRotation3d().getZ()) - angle.getRadians());
    }

    public Pose2d getPose() {
        return swerveDrive.getPose();
    }

    public SwerveDrive getSwerveDrive() {
        return swerveDrive;
    }

    public void resetGyro() {
        swerveDrive.setGyro(new Rotation3d(0, 0, 0));
    }

    public double getMaximumChassisVelocity() {
        return swerveDrive.getMaximumChassisVelocity();
    }

    public double getMaximumChassisAngularVelocity() {
        return swerveDrive.getMaximumChassisAngularVelocity();
    }

    // unsure
    public Rotation2d getYaw() {
        return swerveDrive.getYaw();
    }

    public void resetPose(Pose2d initialPose) {
        swerveDrive.resetOdometry(initialPose);
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return swerveDrive.getRobotVelocity();
    }

    public void driveRobotRelative(ChassisSpeeds chassisSpeeds) {
        swerveDrive.drive(chassisSpeeds);
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

    public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity) {
        return run(() -> {
            swerveDrive.driveFieldOriented(velocity.get());
        });
    }

    public void driveFieldOriented(ChassisSpeeds velocity) {
        swerveDrive.driveFieldOriented(velocity);

    }

    public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier headingX,
            DoubleSupplier headingY) {
        // swerveDrive.setHeadingCorrection(true); // Normally you would want heading
        // correction for this kind of control.
        return run(() -> {

            Translation2d scaledInputs = SwerveMath.scaleTranslation(new Translation2d(translationX.getAsDouble(),
                    translationY.getAsDouble()), 0.8);

            // Make the robot move
            driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(), scaledInputs.getY(),
                    headingX.getAsDouble(),
                    headingY.getAsDouble(),
                    swerveDrive.getOdometryHeading().getRadians(),
                    swerveDrive.getMaximumChassisVelocity()));
        });
    }

    @Override
    public void periodic() {
        // First, tell Limelight your robot's current orientation

        SmartDashboard.putBoolean("isInRange", isInRange());
        SmartDashboard.putBoolean("isApproaching", isApproaching());

        SmartDashboard.putNumber("swerveMaxAngularVelocity", swerveDrive.getMaximumChassisAngularVelocity());
        SmartDashboard.putNumber("swerveAngularVelocity",
                swerveDrive.getRobotVelocity().omegaRadiansPerSecond);
        SmartDashboard.putNumber("swerveRadius", swerveDrive.swerveDriveConfiguration.getDriveBaseRadiusMeters());

        double robotYaw = Units.radiansToDegrees(swerveDrive.getGyro().getRotation3d().getZ());

        LimelightHelpers.SetRobotOrientation("limelight-front", robotYaw, 0.0, 0.0, 0.0, 0.0, 0.0);
        LimelightHelpers.SetRobotOrientation("limelight-back", robotYaw, 0.0, 0.0, 0.0, 0.0, 0.0);

        // Get the pose estimate
        LimelightHelpers.PoseEstimate frontLimelightMeasurement = LimelightHelpers
                .getBotPoseEstimate_wpiBlue_MegaTag2("limelight-front");
        LimelightHelpers.PoseEstimate backLimelightMeasurement = LimelightHelpers
                .getBotPoseEstimate_wpiBlue_MegaTag2("limelight-back");

        // Add it to your pose estimator
        swerveDrive.setVisionMeasurementStdDevs(VecBuilder.fill(.5, .5, 9999999));

        if (frontLimelightMeasurement != null && frontLimelightMeasurement.tagCount > 0) {
            if (frontLimelightMeasurement.pose.getX() >= 0 ||
                    frontLimelightMeasurement.pose.getY() >= 0) {
                swerveDrive.addVisionMeasurement(
                        frontLimelightMeasurement.pose,
                        frontLimelightMeasurement.timestampSeconds);
                double[] frontLimelightEstimatedPose = {
                        frontLimelightMeasurement.pose.getX(),
                        frontLimelightMeasurement.pose.getY() };
                SmartDashboard.putNumberArray("frontLimelightEstimatedPose",
                        frontLimelightEstimatedPose);
                SmartDashboard.putNumber("frontLimelightAveragedist",
                        LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-front").avgTagDist);
            }
        }
        if (backLimelightMeasurement != null && backLimelightMeasurement.tagCount > 0) {
            if (backLimelightMeasurement.pose.getX() >= 0 ||
                    backLimelightMeasurement.pose.getY() >= 0) {
                swerveDrive.addVisionMeasurement(
                        backLimelightMeasurement.pose,
                        backLimelightMeasurement.timestampSeconds);
                double[] backLimelightEstimatedPose = { backLimelightMeasurement.pose.getX(),
                        backLimelightMeasurement.pose.getY() };
                SmartDashboard.putNumberArray("backLimelightEstimatedPose",
                        backLimelightEstimatedPose);
                SmartDashboard.putNumber("backLimelightAveragedist",
                        LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-back").avgTagDist);
            }
        }

        swerveDrive.updateOdometry();
        m_field.setRobotPose(getPose());
        SmartDashboard.putString("swerveModules", swerveDrive.getModules().toString());

        SmartDashboard.putBoolean("allianceIsBlue", DriverStation.getAlliance().get() == DriverStation.Alliance.Blue);
        SmartDashboard.putNumber("rotationToHub", getRotationToPoint(getHubPoint()));

    }
}
