package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralHoldAngleSubsystemConstants;

public class CoralHoldAngleSubsystem extends SubsystemBase {

  private final SparkFlex motor;
  private final SparkFlexConfig motorConfig;

  private double wantedPos;

  SparkClosedLoopController controller;

  private boolean buttonPressed;

  // The endcoder isn't used in the basic form of the subsystem - But we may need it later on
  // would need to add 'import edu.wpi.first.wpilibj.Encoder;' if we do
  private RelativeEncoder encoder;

  public CoralHoldAngleSubsystem() {
    motor =
        new SparkFlex(
            CoralHoldAngleSubsystemConstants.CORAL_HOLD_ANGLE_MOTOR_ID,
            MotorType.kBrushless); // Assign motor controller port
    motorConfig = new SparkFlexConfig();
    controller = motor.getClosedLoopController();
    encoder = motor.getEncoder();

    motorConfig.idleMode(IdleMode.kBrake);
    motorConfig.closedLoop.pidf(0.5f, 0.5f, 0.5f, 0.5f, ClosedLoopSlot.kSlot0);

    // encoder = new Encoder(1, 1); // Assign encoder ports
    wantedPos = encoder.getPosition();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("CoralEncoder:", encoder.getPosition());
  }

  private double setSpeed() {
    double kP = 0.1;
    return kP * 1; // Based around elevator's 'calculate speed.' Will be adjusted later on.
  }

  // Free hand tilt down. Just hold a button and go. Need an if statement
  public void tiltedDown() {
    buttonPressed = true;

    motorConfig.inverted(true);
    motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    wantedPos -= 0.05;

    controller.setReference(
      wantedPos, ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  // Free hand tilt up. Just hold a button and go. Need an if statement
  public void tiltUp() {
    buttonPressed = true;

    motorConfig.inverted(false);
    motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    wantedPos += 0.1;

    controller.setReference(
      wantedPos, ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  public double getEncoder() {
    return motor.getEncoder().getPosition();
  }

  /*public void holdUp() {
    // Calculate the feedforward from the sepoint
    // double feedforward = m_feedforward.calculate(wantedPos, encoder.getVelocity());

    // motorConfig.closedLoop.velocityFF(feedforward);
    controller.setReference(wantedPos, ControlType.kPosition, ClosedLoopSlot.kSlot1);
    // Add the feedforward to the PID output to get the motor output
    /*maxPid.setReference(
    wantedPos, // - ArmConstants.kArmOffsetRads, 0, feedforward
    ControlType.kPosition);

  } */

  public void holdUp(double wantedPos) {
    // motorConfig.closedLoop.velocityFF(feedforward);
    controller.setReference(0, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
    // Add the feedforward to the PID output to get the motor output
    /*maxPid.setReference(
    wantedPos, // - ArmConstants.kArmOffsetRads, 0, feedforward
    ControlType.kPosition);*/

  }

  // When you release a button, this is called to stop the tilt.
  public void stopTilt() {
    motor.set(0);
    buttonPressed = false;
  }
}
