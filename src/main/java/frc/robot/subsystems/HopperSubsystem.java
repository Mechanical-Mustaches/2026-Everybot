package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HopperSubsystem extends SubsystemBase {
    private SparkMax conveyorMotor = new SparkMax(13, MotorType.kBrushless);
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
        leftServo.set(1);
        rightServo.set(-1);

    }

    public void latch() {
        leftServo.set(-1);
        rightServo.set(1);

    }

    public void stopServos() {
        leftServo.set(0);
        rightServo.set(0);

    }

}
