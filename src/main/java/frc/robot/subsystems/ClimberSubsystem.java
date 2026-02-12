
package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class ClimberSubsystem implements Subsystem {

        private static int EXTEND_RANGE = -1;
        private static int CLIMB_RANGE = 1;

        private static double MAIN_TOLERANCE = 0.1;

        private SparkMax climber = new SparkMax(10, MotorType.kBrushed);
        private SparkMaxConfig climberConfig;
        private ClosedLoopConfig climberClosedLoopConfig;
        private SparkClosedLoopController closedLoopController = climber.getClosedLoopController();

        public ClimberSubsystem() {
                // main climber configuration:
                climberConfig = new SparkMaxConfig();

                // TODO: Update PID constants
                climberClosedLoopConfig = new ClosedLoopConfig()
                                .pid(0.1, 0, 0)
                                .outputRange(EXTEND_RANGE, CLIMB_RANGE)
                                .feedbackSensor(FeedbackSensor.kPrimaryEncoder);

                climberConfig
                                // TODO: Find range of rotations needed
                                .smartCurrentLimit(40)
                                .idleMode(IdleMode.kBrake)// kBrake prevent motor from moving when force is applied
                                .apply(climberClosedLoopConfig);

                climber.configure(climberConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        }

        public void extend(int stage) {

                if (stage == 0) { 
                        var controller = climber.getClosedLoopController();
                        controller.setSetpoint(EXTEND_RANGE, ControlType.kPosition);
                        
                }

        }

        public void retract(int stage) {
                if (stage == 0) {
                        closedLoopController.setSetpoint(CLIMB_RANGE, ControlType.kPosition);
                        
                }
        }

        public boolean isExtended(int stage) {// mehtod cheks if the motor in extended position

                if (stage == 0) {
                        var position = climber.getEncoder().getPosition();
                        return position < EXTEND_RANGE + MAIN_TOLERANCE
                                        || position > EXTEND_RANGE - MAIN_TOLERANCE;// adds range for motor to return
                                                                                    // its position

                }
                return false;
        }

        public boolean isRetracted(int stage) {// mehtod cheks if the motor in retracted position
                if (stage == 0) {
                        var position = climber.getEncoder().getPosition();
                        return position < EXTEND_RANGE + MAIN_TOLERANCE
                                        || position > EXTEND_RANGE - MAIN_TOLERANCE;// adds range for motor to return
                                                                                    // its position
                }
                return false;
        }

        public void stop() {
                var controller = climber.getClosedLoopController();
                controller.setSetpoint(0, ControlType.kPosition);
        }

}