package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralHoldAngleSubsystem extends SubsystemBase {

  private final SparkFlex motor;

  // The endcoder isn't used in the basic form of the subsystem - But we may need it later on
  // would need to add 'import edu.wpi.first.wpilibj.Encoder;' if we do
  // private final Encoder encoder;

  public CoralHoldAngleSubsystem() {
    motor = new SparkFlex(1, MotorType.kBrushless); // Assign motor controller port

    // encoder = new Encoder(1, 1); // Assign encoder ports
  }

  private double setSpeed() {
    double kP = 0.01;
    return kP * 1; // Based around elevator's 'calculate speed.' Will be adjusted later on.
  }

  // Free hand tilt down. Just hold a button and go. Need an if statement
  public void tiltedDown() {

    motor.set(setSpeed() * -1); // reverses the motor. Might break it.
  }

  // Free hand tilt up. Just hold a button and go. Need an if statement
  public void tiltUp() {

    motor.set(setSpeed());
  }

  // When you release a button, this is called to stop the tilt.
  public void stopTilt() {
    motor.set(0);
  }
}
