package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HopperSubsystem extends SubsystemBase {
    private SparkMax conveyorMotor = new SparkMax(13, MotorType.kBrushless);

    public HopperSubsystem(SparkMax conveyorMotor) {

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
