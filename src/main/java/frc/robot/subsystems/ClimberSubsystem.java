package frc.robot.subsystems;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SoftLimitConfig;

import edu.wpi.first.wpilibj2.command.Subsystem;

public class ClimberSubsystem implements Subsystem {

    private static int MAIN_EXTEND_RANGE = 1;
    private static int SECONDARY_EXTEND_RANGE = -1;

    private static int MAIN_CLIMB_RANGE = 10;
    private static int SECONDARY_CLIMB_RANGE = -10;

    private static double MAIN_TOLERANCE = 0.7;
    private static double SECONDARY_TOLERANCE = 0.7;

    private SparkMax mainClimber;

    private SparkMaxConfig mainClimberConfig;

    private ClosedLoopConfig mainClimberClosedLoopConfig;

    public ClimberSubsystem() {
        // main climber configuration:

        // TODO: Get motor ID
        mainClimber = new SparkMax(10, MotorType.kBrushed);

        mainClimberConfig = new SparkMaxConfig();

        // TODO: Update PID constants
        mainClimberClosedLoopConfig = new ClosedLoopConfig()
                .pid(0.1, 0, 0)
                .outputRange(MAIN_EXTEND_RANGE, MAIN_CLIMB_RANGE)
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder);

        SoftLimitConfig softLimits = new SoftLimitConfig()
                .forwardSoftLimit(SECONDARY_CLIMB_RANGE)/// set to -10 rotations
                .reverseSoftLimit(SECONDARY_EXTEND_RANGE)// set to 1 torations
                .forwardSoftLimitEnabled(true)
                .reverseSoftLimitEnabled(true);

        mainClimberConfig
                // TODO: Find range of rotations needed
                .smartCurrentLimit(40)
                .idleMode(IdleMode.kBrake)// kBrake prevent motor from moving when force is applied
                .apply(mainClimberClosedLoopConfig)
                .apply(softLimits);

        mainClimber.configure(mainClimberConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // TODO: get real motor ID

    }

    public boolean isExtended(int stage) {
        double position = mainClimber.getEncoder().getPosition();

        if (stage == 0) {
            return Math.abs(position - MAIN_EXTEND_RANGE) <= MAIN_TOLERANCE;
        }
        if (stage == 1) {
            return Math.abs(position - SECONDARY_EXTEND_RANGE) <= SECONDARY_TOLERANCE;
        }
        return false;
    }

    public boolean isRetracted(int stage) {
        double position = mainClimber.getEncoder().getPosition();

        if (stage == 0) {
            return Math.abs(position - MAIN_CLIMB_RANGE) <= MAIN_TOLERANCE;
        }
        if (stage == 1) {
            return Math.abs(position - SECONDARY_CLIMB_RANGE) <= SECONDARY_TOLERANCE;
        }
        return false;
    }

    public void extend(int stage) {
        var controller = mainClimber.getClosedLoopController();

        if (stage == 0) {
            controller.setReference(MAIN_EXTEND_RANGE, ControlType.kPosition);

        } else if (stage == 1) {
            controller.setReference(SECONDARY_EXTEND_RANGE, ControlType.kPosition);
        }
    }

    public void retract(int stage) {
        var controller = mainClimber.getClosedLoopController();

        if (stage == 0) {
            controller.setReference(MAIN_CLIMB_RANGE, ControlType.kPosition);

        } else if (stage == 1) {
            controller.setReference(SECONDARY_CLIMB_RANGE, ControlType.kPosition);
        }
    }

    public void stop() {
        mainClimber.stopMotor();
    }

}
