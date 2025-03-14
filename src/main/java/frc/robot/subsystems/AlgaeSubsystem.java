package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeSubsystem extends SubsystemBase {

  private final SparkFlex motor;
  private final SparkFlexConfig motorConfig;

  // private final Encoder encoder;

  public AlgaeSubsystem() {
    motor =
        new SparkFlex(
            frc.robot.Constants.AlgaeSubsystemConstants.ALGAE_MOTOR_ID, MotorType.kBrushless);
    motorConfig = new SparkFlexConfig();

    // encoder = new Encoder(1, 1);
  }

  /**
   * This method returns a constant speed for now. Adjust this to take joystick input or PID control
   * later on.
   */
  private double setSpeed() {
    double kP = 0.3; // Tweak as necessary.
    return kP * 1; // Currently a placeholder for speed calculation
  }

  // This method will be called in the periodic loop for constant operation if needed
  @Override
  public void periodic() {
    // Add any periodic tasks here, such as motor feedback or sensor checks.
  }

  // Shoots the ball (or algae). The claw motor runs in the forward direction.
  public void shoot() {
    motorConfig.inverted(false);
    motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    motor.set(-setSpeed()); // Run motor forward to shoot the ball (algae)
  }

  // Intakes the ball (algae). The claw motor runs in the reverse direction.
  public void intake() {
    motorConfig.inverted(true); // Invert motor direction for intake
    motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    motor.set(setSpeed()); // Reverse the motor to pull the algae into the claw
  }

  // Stops the motor. Useful for stopping the claw at any time.
  public void stop() {
    motor.set(0); // Completely stop motor when the claw doesn't need to run
  }
}
