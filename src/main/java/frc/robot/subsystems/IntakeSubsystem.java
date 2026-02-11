package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj2.command.Subsystem;

public class IntakeSubsystem implements Subsystem {

    SparkMax mainMotor = new SparkMax(12, MotorType.kBrushless);
    SparkMax mainMotorFollower = new SparkMax(11, MotorType.kBrushless);
    SparkMax indexMotor = new SparkMax(9, MotorType.kBrushless);

    SparkMaxConfig mainMotorFollowerConfig = new SparkMaxConfig();

    public IntakeSubsystem() {
        mainMotorFollowerConfig.follow(12);
        mainMotorFollower.configure(mainMotorFollowerConfig, ResetMode.kNoResetSafeParameters,
                PersistMode.kNoPersistParameters);

    }

    public void shoot() {
        mainMotor.set(1);
    }

    public void indexIn() {
        indexMotor.set(1);
    }

    public void indexOut() {
        indexMotor.set(-1);
    }

    public void stopShooter() {
        mainMotor.set(0);
    }

    public void stopIndex() {
        indexMotor.set(0);
    }

}