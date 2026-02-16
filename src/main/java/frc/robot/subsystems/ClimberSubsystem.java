package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class ClimberSubsystem implements Subsystem {

    private SparkMax mainClimber = new SparkMax(9, MotorType.kBrushed);
    private SparkMaxConfig mainClimberConfig = new SparkMaxConfig();
    private ClosedLoopConfig mainClimberClosedLoopConfig;

     public enum Stage{
        S1(2000),
        S2(4000);
  
        public final double encoderValue;

        private Stage(double stage){
            this.encoderValue = stage;
        }

    }

    public ClimberSubsystem() {

        mainClimberClosedLoopConfig = new ClosedLoopConfig()
                .pid(0.1, 0, 0)
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder);


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
        SmartDashboard.putNumber("climberEncoder", position);
    }

    public void unClimb() {
        double target = Stage.S2.encoderValue;
        double position = mainClimber.getEncoder().getPosition();
        if (position < target) {
            mainClimber.set(-.25);
        }
    }

    public void dumbClimb(){
        mainClimber.set(.25);
        SmartDashboard.putNumber("climberEncoder", mainClimber.getEncoder().getPosition());
    }
    public void dumbUnClimb(){
        mainClimber.set(-.25);
        SmartDashboard.putNumber("climberEncoder", mainClimber.getEncoder().getPosition());
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("climberEncoder", mainClimber.getEncoder().getPosition());
    }

  public void stop(){
    mainClimber.set(0);
  }
}