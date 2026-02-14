package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.Subsystem;

public class ClimberSubsystem implements Subsystem {

    private SparkMax mainClimber = new SparkMax(9, MotorType.kBrushed);
    private SparkMaxConfig mainClimberConfig = new SparkMaxConfig();

    public ClimberSubsystem() {

        mainClimberConfig
                .smartCurrentLimit(40)
                .idleMode(IdleMode.kBrake);// kBrake prevent motor from moving when force is applied

        mainClimber.configure(mainClimberConfig, ResetMode.kNoResetSafeParameters,
                PersistMode.kNoPersistParameters);


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
