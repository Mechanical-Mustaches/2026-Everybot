package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HopperSubsystem extends SubsystemBase {
    private SparkMax conveyorMotor = new SparkMax(9, MotorType.kBrushless);
    private Servo leftServo = new Servo(9);
    private Servo rightServo = new Servo(8);

    public HopperSubsystem() {

    }

    public void conveyorIn() {
        conveyorMotor.set(1);
    }

    public void conveyorOut() {
        conveyorMotor.set(-1);
    }

    public void conveyorStop() {
        conveyorMotor.set(0);
    }

    public void unlatch() {
        leftServo.setSpeed(1);
        rightServo.setSpeed(-1);

    }

    public void latch() {
        leftServo.setSpeed(-1);
        rightServo.setSpeed(1);

    }

    public void stopServos() {
        leftServo.setSpeed(0);
        rightServo.setSpeed(-0);

    }

}
