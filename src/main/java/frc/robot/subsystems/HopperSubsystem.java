package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HopperSubsystem extends SubsystemBase {
    private SparkMax conveyorMotor = new SparkMax(9, MotorType.kBrushless);

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

}
