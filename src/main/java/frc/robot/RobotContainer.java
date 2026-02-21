// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ClimberCommand;
import frc.robot.commands.FeedCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.MoveToScoreCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.ClimberSubsystem.Stage;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  public final ClimberSubsystem climberSubsystem;
  public final IntakeSubsystem intakeSubsystem;
  private final SwerveDriveSubsystem swerveDriveSubsystem;
  private final HopperSubsystem hopperSubsystem;

  private final SendableChooser<Command> autoChooser;

  private final CommandXboxController m_driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);

  private final CommandGenericHID m_gunnerController = new CommandGenericHID(OperatorConstants.kGunnerControllerPort);
  // private final XboxController driverController_HID =
  // m_driverController.getHID();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    swerveDriveSubsystem = new SwerveDriveSubsystem();
    climberSubsystem = new ClimberSubsystem();
    intakeSubsystem = new IntakeSubsystem();
    hopperSubsystem = new HopperSubsystem();
    // Configure the trigger bindings

    // NamedCommands.registerCommand("AlignClimb", );
    NamedCommands.registerCommand("Shoot", new ShootCommand(intakeSubsystem, hopperSubsystem));
    NamedCommands.registerCommand("Climb", new ClimberCommand(climberSubsystem, Stage.S1));
    NamedCommands.registerCommand("AlignScore", new MoveToScoreCommand(swerveDriveSubsystem));
    NamedCommands.registerCommand("AlignClimb", new MoveToScoreCommand(swerveDriveSubsystem));

    configureBindings();

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    swerveDriveSubsystem.setDefaultCommand(swerveDriveSubsystem.driveCommand(
        () -> -MathUtil.applyDeadband(m_driverController.getRawAxis(1), 0.1),
        () -> -MathUtil.applyDeadband(m_driverController.getRawAxis(0), 0.1),
        () -> -MathUtil.applyDeadband(m_driverController.getRawAxis(4), 0.1)));

    m_driverController.leftBumper().whileTrue(new MoveToScoreCommand(swerveDriveSubsystem));
    m_driverController.rightBumper().onTrue(new InstantCommand(() -> swerveDriveSubsystem.resetGyro()));

    m_gunnerController.button(4).whileTrue(new ShootCommand(intakeSubsystem, hopperSubsystem));
    m_gunnerController.button(1).whileTrue(new IntakeCommand(intakeSubsystem, hopperSubsystem));
 //   m_gunnerController.button(3).whileTrue(new FeedCommand(intakeSubsystem));

    m_gunnerController.button(2).whileTrue(new ClimberCommand(climberSubsystem, Stage.S4));
    m_gunnerController.button(3).whileTrue(new ClimberCommand(climberSubsystem, Stage.S2));
    m_gunnerController.button(5).whileTrue(new ClimberCommand(climberSubsystem, Stage.S3));
    m_gunnerController.button(6).whileTrue(new ClimberCommand(climberSubsystem, Stage.S1));

    m_gunnerController.button(11).onTrue(new InstantCommand(() -> climberSubsystem.dumbClimb()));
    m_gunnerController.button(11).onFalse(new InstantCommand(() -> climberSubsystem.stop()));
    m_gunnerController.button(12).onTrue(new InstantCommand(() -> climberSubsystem.dumbUnClimb()));
    m_gunnerController.button(12).onFalse(new InstantCommand(() -> climberSubsystem.stop()));
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
