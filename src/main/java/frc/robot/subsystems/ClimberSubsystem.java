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
        mainClimber = new SparkMax(9, MotorType.kBrushed);

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


    }
    public void climb(){
        mainClimber.set(.25);
    }

    public void unclimb(){
        mainClimber.set(-.25);
    }

    public void stop() {
        mainClimber.set(0);
    }

}
