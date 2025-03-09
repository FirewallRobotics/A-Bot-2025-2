package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralHoldSubsystemConstants;

public class CoralHoldSubsystem extends SubsystemBase {

  private final SparkMax motor;
  DigitalInput limitSwitch = new DigitalInput(0);

  // The endcoder isn't used in the basic form of the subsystem - But we may need it later on
  // would need to add 'import edu.wpi.first.wpilibj.Encoder;' if we do
  // private final Encoder encoder;

  public CoralHoldSubsystem() {
    motor =
        new SparkMax(
            CoralHoldSubsystemConstants.CORAL_HOLD_MOTOR_ID,
            MotorType.kBrushless); // Assign motor controller port
    new SparkFlexConfig();

    // encoder = new Encoder(1, 1); // Assign encoder ports
  }

  private double setSpeed() {
    double kP = 0.01;
    return kP * 1; // Based around elevator's 'calculate speed.' Will be adjusted later on.
  }

  // A set of code that constantly goes.
  /*public void periodic(){

    //When shooter is active
      //Let it go if limit switch isn't hit
    //When intake is active
      //Let it go if limit switch is hit
  }*/

  // When we want to shoot coral from the intake, has to have a coral in the lift
  public void shoot() {
    // if (limitSwitch.get()) {
    // motorConfig.inverted(false);
    // motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    motor.set(setSpeed());
    // }
  }

  // Intakes coral. Lift has to be empty
  public void intake() {

    if (limitSwitch.get() == false) {
      // motorConfig.inverted(true);
      // motor.configure(motorConfig, ResetMode.kResetSafeParameters,
      // PersistMode.kPersistParameters);

      motor.set(
          -setSpeed()); // Reverses the motor- Check if to make sure this won't break the motor
    } else {
      stop();
    }
  }

  // Makes the motor stop. Can shut down both functions.
  public void stop() {
    motor.set(0);
  }
}
