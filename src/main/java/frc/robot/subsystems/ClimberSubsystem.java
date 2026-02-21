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

    int revolutions = 0;
    double previousPose = 0;

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
        S2(0.966), //1 revolution
        S3(.150), 
        S4(.424); //???

        public final double encoderValue;

        private Stage(double stage) {
            this.encoderValue = stage;
        }

    }

    public ClimberSubsystem() {

        mainClimberConfig
                .smartCurrentLimit(40)
                .idleMode(IdleMode.kBrake);

        mainClimber.configure(mainClimberConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void reverseClimb() {
       mainClimber.set(-0.25);
    }

    public void climb() {
        mainClimber.set(0.25);
    }

    public void dumbClimb() {
        mainClimber.set(.4);
    }

    public void dumbUnClimb() {
        mainClimber.set(-.4);
    }

    public void stop() {
        mainClimber.set(0);
    }

    public boolean isDone(Stage stage) {
        if ((stage == Stage.S1 && revolutions == 0) || (stage != Stage.S1 && revolutions == 1)){
            if (Math.abs(mainClimber.getAbsoluteEncoder().getPosition() - stage.encoderValue) <= 0.005) {
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
        
        double currentPose = mainClimber.getAbsoluteEncoder().getPosition();
        if (currentPose < 0.1 && previousPose > 0.9){
            revolutions -= 1;
        } else if (currentPose > 0.9 && previousPose < 0.1){
            revolutions += 1;
        }
        SmartDashboard.putNumber("revolutions", revolutions);
        previousPose = currentPose;
    }

    public double getEncoderPosition(){
        return mainClimber.getEncoder().getPosition();
    }
}