package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.fasterxml.jackson.databind.util.ExceptionUtil;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HopperSubsystem extends SubsystemBase {
    private SparkMax conveyorMotor = new SparkMax(9, MotorType.kBrushless);
    private Servo leftServo = new Servo(9);
    private Servo rightServo = new Servo(8);

    public HopperSubsystem() {
        rightServo.setBoundsMicroseconds(2500, 2300, 1500, 700, 500);

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
        var unlatchPosition = 0.5;
        leftServo.set(unlatchPosition - 0.175);
        rightServo.set(unlatchPosition - 0.175);

    }

    public void stopServos() {
        leftServo.set(leftServo.getPosition());
        rightServo.set(rightServo.getPosition());

    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("leftServoPos", leftServo.getPosition());
        SmartDashboard.putNumber("rightServoPos", rightServo.getPosition());
    }

}
