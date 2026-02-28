package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {

    private SparkMax mainClimber = new SparkMax(9, MotorType.kBrushed);
    private SparkMaxConfig mainClimberConfig = new SparkMaxConfig();

    int revolutions = 1;
    double previousPose = 0;
    double mainStageTolerance = 0.015;
    double stage4Tolerance = 0.005;

    /**
     * Depicts the stage the climber should travel to
     * <p>
     * - S1 -> Pre-climb position (horizontal)
     * <p>
     * - S2 -> Neutral position (vertical)
     * <p>
     * - S3 -> L1 Climb position
     * <p>
     * - S4 -> L2 Climb position
     */
    public enum Stage {
        S1(0.775),
        S2(0.886), // 1 revolution
        S3(.80), // previously 0.450
        S4(.8);

        public final double encoderValue;

        private Stage(double stage) {
            this.encoderValue = stage;
        }

    }

    Stage stage = Stage.S1;

    public ClimberSubsystem() {

        mainClimberConfig
                .smartCurrentLimit(40)
                .idleMode(IdleMode.kBrake);

        mainClimber.configure(mainClimberConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void reverseClimb() {
        mainClimber.set(-0.75);
    }

    public void climb() {
        if (revolutions == 1 && Stage.S4.encoderValue - mainClimber.getAbsoluteEncoder().getPosition() <= 0.1) {
            mainClimber.set(0.4);
        } else {
            mainClimber.set(0.75);
        }
    }

    public void dumbClimb() {
        mainClimber.set(.5);
    }

    public void dumbUnClimb() {
        mainClimber.set(-.5);
    }

    public void stop() {
        mainClimber.set(0);
    }

    public boolean isDone(Stage stage) {
        this.stage = stage;
        if (stage == Stage.S2 && revolutions == 0) {
            return true;
        } else if ((stage == Stage.S1 && revolutions == 0) || (stage != Stage.S1 && revolutions == 1)) {
            if (stage != Stage.S4
                    && Math.abs(
                            mainClimber.getAbsoluteEncoder().getPosition() - stage.encoderValue) <= mainStageTolerance
                    ||
                    stage == Stage.S4 && Math.abs(
                            mainClimber.getAbsoluteEncoder().getPosition() - stage.encoderValue) <= stage4Tolerance) {
                return true;
            } else {
                return false;
            }
        } else {
            return false;
        }
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("climberAbsoluteEncoder", mainClimber.getAbsoluteEncoder().getPosition());
        SmartDashboard.putNumber("climberDistanceToTarget",
                Math.abs(mainClimber.getAbsoluteEncoder().getPosition() - stage.encoderValue));
        SmartDashboard.putBoolean("climberIsFinished", isDone(stage));

        double currentPose = mainClimber.getAbsoluteEncoder().getPosition();
        if (currentPose < 0.05 && previousPose > 0.95) {
            revolutions -= 1;
        } else if (currentPose > 0.95 && previousPose < 0.05) {
            revolutions += 1;
        }
        SmartDashboard.putNumber("revolutions", revolutions);
        previousPose = currentPose;
    }

    public double getEncoderPosition() {
        return mainClimber.getEncoder().getPosition();
    }
}