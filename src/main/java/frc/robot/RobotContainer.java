// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ClimberCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.MoveToScoreCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.SpinUpToShootCommandGroup;
import frc.robot.commands.SpitCommand;
import frc.robot.commands.StopCommand;
import frc.robot.commands.AutoCommands.ShootAllCommandGroup;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import swervelib.SwerveInputStream;
import frc.robot.subsystems.ClimberSubsystem.Stage;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

    public final ClimberSubsystem climberSubsystem;
    public final IntakeSubsystem intakeSubsystem;
    private final SwerveDriveSubsystem swerveDriveSubsystem;
    private final HopperSubsystem hopperSubsystem;

    private final SendableChooser<Command> autoChooser;

    private final CommandXboxController m_driverController = new CommandXboxController(
            OperatorConstants.kDriverControllerPort);

    private final CommandXboxController m_pitController = new CommandXboxController(
            OperatorConstants.kPitControllerPort);

    private final CommandGenericHID m_gunnerController = new CommandGenericHID(OperatorConstants.kGunnerControllerPort);

    /**
     * The container for the robot. Contains subsystems, OI devices, and
     * commands.
     */
    public RobotContainer() {

        swerveDriveSubsystem = new SwerveDriveSubsystem();
        climberSubsystem = new ClimberSubsystem();
        intakeSubsystem = new IntakeSubsystem();
        hopperSubsystem = new HopperSubsystem();

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

        // Configure the trigger bindings
        NamedCommands.registerCommand("Shoot", new ShootAllCommandGroup(intakeSubsystem, hopperSubsystem));
        NamedCommands.registerCommand("Climb", new ClimberCommand(climberSubsystem, Stage.S3));
        NamedCommands.registerCommand("PreClimb", new ClimberCommand(climberSubsystem, Stage.S1));
        NamedCommands.registerCommand("Intake", new IntakeCommand(intakeSubsystem, hopperSubsystem));
        NamedCommands.registerCommand("Set Initial Pose",
                new InstantCommand(() -> swerveDriveSubsystem
                .resetPose(new PathPlannerAuto(autoChooser.getSelected().getName()).getStartingPose())));

        configureBindings();

    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be
     * created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor
     * with an arbitrary predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for      {@link
   * CommandXboxController
     * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or      {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
     */
    private void configureBindings() {
        SwerveInputStream driveAngularVelocity;

        if (!m_driverController.leftBumper().getAsBoolean()) {
            driveAngularVelocity = SwerveInputStream.of(swerveDriveSubsystem.getSwerveDrive(),
                    () -> m_driverController.getLeftY() * -1,
                    () -> m_driverController.getLeftX() * -1)
                    .withControllerRotationAxis(m_driverController::getRightX)
                    .deadband(0.1)
                    .scaleTranslation(0.8)
                    .allianceRelativeControl(true);
        } else {
            driveAngularVelocity = SwerveInputStream.of(swerveDriveSubsystem.getSwerveDrive(),
                    () -> m_driverController.getLeftY() * -1,
                    () -> m_driverController.getLeftX() * -1)
                    .withControllerRotationAxis(
                            () -> (swerveDriveSubsystem.getRotationToPoint(SwerveDriveSubsystem.getHubPoint()) / Math.PI))
                    .deadband(0.1)
                    .scaleTranslation(0.8)
                    .allianceRelativeControl(true);
        }

        driveAngularVelocity = SwerveInputStream.of(swerveDriveSubsystem.getSwerveDrive(),
                () -> m_driverController.getLeftY() * -1,
                () -> m_driverController.getLeftX() * -1)
                .withControllerRotationAxis(() -> {
                    if (!m_driverController.leftBumper().getAsBoolean()) {
                        return m_driverController.getRightX();
                    } else {
                        var swerveInput = swerveDriveSubsystem.getRotationToPoint(SwerveDriveSubsystem.getHubPoint())
                                / Math.PI;

                        if (0.4 <= Math.abs(swerveInput) && Math.abs(swerveInput) <= 0.035) {
                            return 0.1;
                        }

                        return swerveInput;
                    }
                })
                .deadband(0.1)
                .scaleTranslation(0.8)
                .allianceRelativeControl(true);

        Command driveFieldOrientedAngularVelocityCommand = swerveDriveSubsystem.driveFieldOriented(driveAngularVelocity);
        swerveDriveSubsystem.setDefaultCommand(driveFieldOrientedAngularVelocityCommand);

        if (DriverStation.isTest()) {
            m_driverController.povRight()
                    .onTrue(new InstantCommand(() -> swerveDriveSubsystem.resetPose(new Pose2d(5, 5, new Rotation2d()))));
        }
        // m_driverController.leftBumper().whileTrue(new
        // MoveToScoreCommand(swerveDriveSubsystem));
        m_driverController.rightBumper().onTrue(new InstantCommand(() -> swerveDriveSubsystem.resetGyro()));

        m_gunnerController.button(1).whileTrue(new IntakeCommand(intakeSubsystem, hopperSubsystem));
        m_gunnerController.button(8).onTrue(new SpinUpToShootCommandGroup(intakeSubsystem, hopperSubsystem, false)
                .until(() -> !m_gunnerController.button(8).getAsBoolean()));
        m_gunnerController.button(4)
                .onFalse(new StopCommand(intakeSubsystem, hopperSubsystem));

        m_gunnerController.button(12).whileTrue(new ClimberCommand(climberSubsystem, Stage.S4));
        m_gunnerController.button(9).whileTrue(new ClimberCommand(climberSubsystem, Stage.S3));
        m_gunnerController.button(6).whileTrue(new ClimberCommand(climberSubsystem, Stage.S2));
        m_gunnerController.button(3).whileTrue(new ClimberCommand(climberSubsystem, Stage.S1));

        m_gunnerController.button(5).onTrue(new InstantCommand(() -> hopperSubsystem.conveyorIn()));
        m_gunnerController.button(5).onFalse(new InstantCommand(() -> hopperSubsystem.conveyorStop()));
        m_gunnerController.button(4).onTrue(new InstantCommand(() -> hopperSubsystem.conveyorOut()));
        m_gunnerController.button(4).onFalse(new InstantCommand(() -> hopperSubsystem.conveyorStop()));

        m_gunnerController.button(7).onTrue(new InstantCommand(() -> intakeSubsystem.reverseIntake()));
        m_gunnerController.button(7).onFalse(new InstantCommand(() -> intakeSubsystem.stopShooter()));

        m_gunnerController.button(2).whileTrue(new SpitCommand(intakeSubsystem));

        m_pitController.povLeft().onTrue(new InstantCommand(() -> climberSubsystem.dumbClimb()));
        m_pitController.povLeft().onFalse(new InstantCommand(() -> climberSubsystem.stop()));
        m_pitController.povRight().onTrue(new InstantCommand(() -> climberSubsystem.dumbUnClimb()));
        m_pitController.povRight().onFalse(new InstantCommand(() -> climberSubsystem.stop()));
        m_pitController.povUp()
                .whileTrue(new RunCommand(() -> swerveDriveSubsystem.driveRobotRelative(new ChassisSpeeds(1, 0, 0))));
        m_pitController.povUp()
                .onFalse(new InstantCommand(() -> swerveDriveSubsystem.driveRobotRelative(new ChassisSpeeds(0, 0, 0))));
        m_driverController.povUp()
                .whileTrue(new RunCommand(() -> swerveDriveSubsystem.driveRobotRelative(new ChassisSpeeds(1, 0, 0))));
        m_driverController.povUp()
                .onFalse(new InstantCommand(() -> swerveDriveSubsystem.driveRobotRelative(new ChassisSpeeds(0, 0, 0))));

        // TEMPORARY
        m_driverController.leftTrigger().whileTrue(new IntakeCommand(intakeSubsystem, hopperSubsystem));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        if (this.autoChooser != null) {
            return autoChooser.getSelected();
        } else {
            return null;
        }

    }

}
