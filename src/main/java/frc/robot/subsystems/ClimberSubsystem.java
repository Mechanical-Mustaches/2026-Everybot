package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {

    private SparkMax mainClimber = new SparkMax(9, MotorType.kBrushed);
    private SparkMaxConfig mainClimberConfig = new SparkMaxConfig();
    private ClosedLoopConfig mainClimberClosedLoopConfig;

    /**
     * Depicts the stage the climber should travel to
     * <p>
     * - S0 -> Horizontal
     * <p>
     * - S1 -> L1 Climb position
     * <p>
     * - S2 -> L2 Climb position
     */
    public enum Stage {
        S0(0),
        S1(.1),
        S2(.2);

        public final double encoderValue;

        private Stage(double stage) {
            this.encoderValue = stage;
        }

    }

    public ClimberSubsystem() {

        mainClimberClosedLoopConfig = new ClosedLoopConfig()
                .pid(0.1, 0, 0);

        mainClimberConfig
                .smartCurrentLimit(40)
                .idleMode(IdleMode.kBrake)
                .apply(mainClimberClosedLoopConfig);

        mainClimber.configure(mainClimberConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void climb() {
        double target = Stage.S1.encoderValue;
        double position = mainClimber.getEncoder().getPosition();
        if (position < target) {
            mainClimber.set(.25);
        }
    }

    public void unClimb() {
        double target = Stage.S2.encoderValue;
        double position = mainClimber.getEncoder().getPosition();
        if (position < target) {
            mainClimber.set(-.25);
        }

        mainClimber.getClosedLoopController().setSetpoint(position, ControlType.kPosition);
    }

    public void dumbClimb() {
        mainClimber.set(.25);
    }

    public void dumbUnClimb() {
        mainClimber.set(-.25);
    }

    public void stop() {
        mainClimber.set(0);
    }

    public boolean isDone(Stage stage) {
        if (mainClimber.getEncoder().getPosition() >= stage.encoderValue) {
            return true;
        } else {
            return false;
        }
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("climberEncoder", mainClimber.getEncoder().getPosition());
        SmartDashboard.putNumber("climberAbsoluteEncoder", mainClimber.getAbsoluteEncoder().getPosition());
    }
}