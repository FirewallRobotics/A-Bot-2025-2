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
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.CoralHoldAngleSubsystemConstants;

public class CoralHoldAngleSubsystem extends SubsystemBase {

  private final SparkFlex motor;
  private final SparkFlexConfig motorConfig;

  private double wantedPos;

  SparkClosedLoopController controller;

  private boolean buttonPressed;

  private TrapezoidProfile.State state;

  private ArmFeedforward feedforward =
      new ArmFeedforward(
          ArmConstants.kSVolts,
          ArmConstants.kGVolts,
          ArmConstants.kVVoltSecondPerRad,
          ArmConstants.kAVoltSecondSquaredPerRad);

  // The endcoder isn't used in the basic form of the subsystem - But we may need it later on
  // would need to add 'import edu.wpi.first.wpilibj.Encoder;' if we do
  private RelativeEncoder encoder;

  public static final double[] angles = {0, 0, 0, 0, 0};

  public CoralHoldAngleSubsystem() {
    motor =
        new SparkFlex(
            CoralHoldAngleSubsystemConstants.CORAL_HOLD_ANGLE_MOTOR_ID,
            MotorType.kBrushless); // Assign motor controller port
    motorConfig = new SparkFlexConfig();
    controller = motor.getClosedLoopController();
    encoder = motor.getEncoder();

    motorConfig.idleMode(IdleMode.kBrake);
    motorConfig.closedLoop.pidf(0f, 0f, 0f, 0f, ClosedLoopSlot.kSlot0);

    // encoder = new Encoder(1, 1); // Assign encoder ports
    wantedPos = encoder.getPosition();
    state = new State(wantedPos, 0);
    // 0.73 L2-L3
    // 0.90 Recieve
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("CoralEncoder:", encoder.getPosition());

    if (!buttonPressed) {
      wantedPos = encoder.getPosition();
      state = new State(wantedPos, 0);
      holdUp(state);
    }
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

    motor.set(setSpeed());
  }

  public void IntakePosition() {
    double setPoint = 0.9;
    if (setPoint + 0.1 < encoder.getPosition()) {
      motorConfig.inverted(false);
      motor.set(setSpeed());
    } else if (setPoint - 0.1 > encoder.getPosition()) {
      motorConfig.inverted(true);
      motor.set(setSpeed());
    } else {
      wantedPos = encoder.getPosition();
      state = new State(wantedPos, 0);
      holdUp(state);
    }
  }

  public void LPosition() {
    State setpoint = new State(0.73, 0);
    double ff = feedforward.calculate(setpoint.position * 2 * Math.PI, setpoint.velocity);
    controller.setReference(0.73, ControlType.kPosition, ClosedLoopSlot.kSlot0, ff);
  }

  // Free hand tilt up. Just hold a button and go. Need an if statement
  public void tiltUp() {
    buttonPressed = true;

    motorConfig.inverted(false);
    motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    motor.set(setSpeed());
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


  public void setLevel(int level) {
    if (level < 0 || level >= angles.length) {
      System.out.println("Invalid level: " + level);
      return;
    }
    wantedPos = angles[level - 1];
    holdUp(wantedPos);
  }

  public boolean isFinished(int position) {

    if (angles[position] - (motor.getEncoder().getPosition()) == 0) {
      return true;
    } else {
      return false;
    }
  }

  public void holdUp(TrapezoidProfile.State setpoint) {
    // motorConfig.closedLoop.velocityFF(feedforward);
    double ff = feedforward.calculate(setpoint.position * 2 * Math.PI, setpoint.velocity);
    controller.setReference(0, ControlType.kPosition, ClosedLoopSlot.kSlot0, ff);
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
