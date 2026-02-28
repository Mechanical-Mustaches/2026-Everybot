package frc.robot.subsystems;

import java.util.ArrayList;

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

    private record AmperageMeasurements(long time, double amperage) {
        public boolean isRecent(long milliseconds) {
            return System.currentTimeMillis() - time <= milliseconds;
        }
    }

    private ArrayList<AmperageMeasurements> amperageMeasurements = new ArrayList<AmperageMeasurements>();

    private static final long MEASUREMENT_WINDOW = 500;
    // this number represents the minimum number of recent measurements needed to
    // calculate average amperage
    private static final int RECENT_MEASUREMENT_COUNT = 5;
    public static final double FUEL_DETECTION_THRESHOLD = 15;

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

    private double getAverageAmperage() {
        if (amperageMeasurements.isEmpty()) {
            return 0;
        }

        var recentMeasurementCount = 0;
        double sumOfElements = 0;

        for (var measurement : amperageMeasurements) {
            sumOfElements = sumOfElements + measurement.amperage;
            if (measurement.isRecent(MEASUREMENT_WINDOW)) {
                recentMeasurementCount = recentMeasurementCount + 1;
            }
        }

        if (recentMeasurementCount < RECENT_MEASUREMENT_COUNT) {
            return 0;
        }

        return sumOfElements / amperageMeasurements.size();
    }

    public boolean isFuelDetected() {
        return getAverageAmperage() > FUEL_DETECTION_THRESHOLD;
    }

    public boolean isAtSpeed() {
        return mainMotor.getEncoder().getVelocity() >= 300;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("FlywheelVelocity", mainMotor.getEncoder().getVelocity());
    }

}