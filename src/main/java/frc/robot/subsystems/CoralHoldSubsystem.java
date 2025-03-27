package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.CoralHoldSubsystemConstants;

public class CoralHoldSubsystem extends SubsystemBase {

  private final SparkMax motor;

  // DigitalInput limitSwitch = new DigitalInput(0);
  DigitalInput limitSwitch = new DigitalInput(0);
  boolean doingStuff = false;

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
    double kP = 0.2;
    return kP * 1; // Based around elevator's 'calculate speed.' Will be adjusted later on.
  }

  // A set of code that constantly goes.
  /*public void periodic(){

    //When shooter is active
      //Let it go if limit switch isn't hit
    //When intake is active
      //Let it go if limit switch is hit
  }*/

  /**
   * When we want to shoot coral from the intake. Moves at negative {@link #setSpeed()} x 1.25
   *
   * @implNote Currently doesn't use the limit switch but can in the future if we have it
   */
  public void shoot() {
    // if (!limitSwitch.get()) {
    // motorConfig.inverted(false);
    // motor.configure(motorConfig, ResetMode.kResetSafeParameters,
    // PersistMode.kPersistParameters);

    motor.set(-setSpeed() * 1.25);
    // }
    if (!limitSwitch.get()) {
      // motorConfig.inverted(false);
      // motor.configure(motorConfig, ResetMode.kResetSafeParameters,
      // PersistMode.kPersistParameters);
      motor.set(-setSpeed() * 1.25);
    }
  }

  /**
   * Intakes coral. Moves at {@link #setSpeed()}
   *
   * @implNote Currently doesn't use the limit switch but can in the future if we have it
   */
  public void intake() {
    // if (limitSwitch.get()) {
    motor.set(setSpeed());
    // } else {
    //  motor.set(0);
    // }
    if (limitSwitch.get()) {
      motor.set(setSpeed());
      doingStuff = true;

    } else if (limitSwitch.get() && doingStuff == true) {
      new WaitCommand(1.2f);
      motor.set(0);
      doingStuff = false;
    }
  }

  /** Makes the motor stop. */
  public void stop() {
    motor.set(0);
    doingStuff = false;
  }

  public boolean getTrigger() {
    return limitSwitch.get();
  }
}
