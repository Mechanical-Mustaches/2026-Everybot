package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

    SparkMax mainMotor = new SparkMax(12, MotorType.kBrushless);
    SparkMax mainMotorFollower = new SparkMax(11, MotorType.kBrushless);
    SparkMax indexMotor = new SparkMax(10, MotorType.kBrushless);

    SparkMaxConfig mainMotorFollowerConfig = new SparkMaxConfig();

    public IntakeSubsystem() {
        mainMotorFollowerConfig.follow(12, true);
        mainMotorFollower.configure(mainMotorFollowerConfig, ResetMode.kNoResetSafeParameters,
                PersistMode.kNoPersistParameters);

    }

    public void shoot() {
        mainMotor.set(-1);
    }

    public void intake() {
        mainMotor.set(-0.5);
    }

    public void indexIn() {
        indexMotor.set(-.75);
    }

    public void indexOut() {
        indexMotor.set(1);
    }

    public void stopShooter() {
        mainMotor.set(0);
    }

    public void reverseIntake() {
        mainMotor.set(1);
    }

    public void stopIndex() {
        indexMotor.set(0);
    }

    public void feed() {
        indexOut();
        shoot();
    }

    public boolean isAtSpeed() {
        return mainMotor.getEncoder().getVelocity() >= 300;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("FlywheelVelocity", mainMotor.getEncoder().getVelocity());
    }

}