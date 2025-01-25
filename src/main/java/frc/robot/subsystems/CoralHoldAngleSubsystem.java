package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralHoldAngleSubsystemConstants;

public class CoralHoldAngleSubsystem extends SubsystemBase {

  private final SparkFlex motor;
  private final SparkFlexConfig motorConfig;

  // The endcoder isn't used in the basic form of the subsystem - But we may need it later on
  // would need to add 'import edu.wpi.first.wpilibj.Encoder;' if we do
  // private final Encoder encoder;

  public CoralHoldAngleSubsystem() {
    motor =
        new SparkFlex(
            CoralHoldAngleSubsystemConstants.CORAL_HOLD_ANGLE_MOTOR_ID,
            MotorType.kBrushless); // Assign motor controller port
    motorConfig = new SparkFlexConfig();
    // encoder = new Encoder(1, 1); // Assign encoder ports
  }

  private double setSpeed() {
    double kP = 0.01;
    return kP * 1; // Based around elevator's 'calculate speed.' Will be adjusted later on.
  }

  // Free hand tilt down. Just hold a button and go. Need an if statement
  public void tiltedDown() {

    motorConfig.inverted(true);
    motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    motor.set(setSpeed()); // reverses the motor. Might break it.
  }

  // Free hand tilt up. Just hold a button and go. Need an if statement
  public void tiltUp() {

    motorConfig.inverted(false);
    motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    motor.set(setSpeed());
  }

  // When you release a button, this is called to stop the tilt.
  public void stopTilt() {
    motor.set(0);
  }
}
