package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralHoldSubsystem extends SubsystemBase {

  private final SparkFlex motor;

  // The endcoder isn't used in the basic form of the subsystem - But we may need it later on
  // would need to add 'import edu.wpi.first.wpilibj.Encoder;' if we do
  // private final Encoder encoder;

  public CoralHoldSubsystem() {
    motor = new SparkFlex(2, MotorType.kBrushless); // Assign motor controller port
    // encoder = new Encoder(1, 1); // Assign encoder ports
  }

  private double setSpeed() {
    double kP = 0.01;
    return kP * 1; // Based around elevator's 'calculate speed.' Will be adjusted later on.
  }

  // A set of code that constantly goes.
  // Not currently being used - Might need it later.
  /*public void periodic(){

  }*/

  // When we want to shoot coral from the intake, has to have a coral in the lift
  public void shoot() {

    motor.set(setSpeed());
  }

  // Intakes coral. Lift has to be empty
  public void intake() {
    motor.set(
        setSpeed() * -1); // Reverses the motor- Check if to make sure this won't break the motor
  }

  // Makes the motor stop. Can shut down both functions.
  public void stop() {
    motor.set(0);
  }
}
