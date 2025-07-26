package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.CoralHoldAngleSubsystemConstants;

public class CoralWristSubsystem extends SubsystemBase {
  // Establish motor and motor config
  private final SparkFlex motor;
  private final SparkFlexConfig motorConfig;

  // Establish encoder
  private final RelativeEncoder encoder;

  // Arm feedforward set up. This is something pre-made to help us keep the arm up
  private final ArmFeedforward feedforward;

  // Sets up the controller that way we can use to hold up and go to locations
  private SparkClosedLoopController controller;

  // if the button is pressed, this is true
  private boolean buttonPressed;

  // sets up wanted pos
  private double wantedPos;

  // sets up the state
  private TrapezoidProfile.State state;

  // Constructor (I always lose this)
  public CoralWristSubsystem() {
    // Define motor and motor config
    motor =
        new SparkFlex(
            CoralHoldAngleSubsystemConstants.CORAL_HOLD_ANGLE_MOTOR_ID, MotorType.kBrushless);

    motorConfig = new SparkFlexConfig();

    // sets up the encoder
    encoder = motor.getEncoder();

    // gets the controller needed
    controller = motor.getClosedLoopController();

    // Set up config
    motorConfig.idleMode(IdleMode.kBrake);
    motorConfig.encoder.positionConversionFactor(1);
    motorConfig.encoder.velocityConversionFactor(1);

    // Config limiter (Important: If we don't have this, automatic will destroy)
    // Have both this and setSpeed functions
    motorConfig.smartCurrentLimit(16);

    // Sets up the arm feedforward
    feedforward =
        new ArmFeedforward(
            ArmConstants.kSVolts,
            ArmConstants.kGVolts,
            ArmConstants.kVVoltSecondPerRad,
            ArmConstants.kAVoltSecondSquaredPerRad);

    // The original PID for manual working
    motorConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pidf(0f, 0f, 0f, 0f, ClosedLoopSlot.kSlot0);

    motor.configure(motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    wantedPos = encoder.getPosition();
    state = new State(wantedPos, 0);
  }

  // Just puts where our current encoder is on the smart dashboard (Yes, I am continuing to use
  // Milo's way despite fighting them on it early. No I will not admit I was wrong.)
  @Override
  public void periodic() {
    SmartDashboard.putNumber("CoralEncoder:", encoder.getPosition());

    if (!buttonPressed) {
      wantedPos = encoder.getPosition();
      state = new State(wantedPos, 0);
      // Logger.getGlobal().log(Level.INFO, "(PERIODIC) Hold up trying to get: " + wantedPos);
      holdUp(state);
    }
  }

  // Tilt up
  public void goUp() {
    buttonPressed = true;
    motor.set(ArmConstants.speed);
  }

  // Tilt down
  public void goDown() {
    buttonPressed = true;
    motor.set(-(ArmConstants.speed / 2));
  }

  // stop tilt
  public void stopWrist() {
    motor.set(0);

    motorConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pidf(0f, 0f, 0f, 0, ClosedLoopSlot.kSlot0);

    motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    State setPoint = new State(getEncoder(), 0);
    holdUp(setPoint);
    buttonPressed = false;
  }

  // Hold up at a position
  public void holdUp(TrapezoidProfile.State setpoint) {
    // Logger.getGlobal().log(Level.INFO, "(FUNCTION) Hold up trying to get: " + setpoint.position);
    double ff = feedforward.calculate(setpoint.position * 2 * Math.PI, setpoint.velocity);
    controller.setReference(0, ControlType.kPosition, ClosedLoopSlot.kSlot0, ff);
  }

  // get encoder
  public double getEncoder() {
    return encoder.getPosition();
  }

  // Automatically move to a point
  public void goToCoralWristLevel(int level) {
    buttonPressed = true;
    double setPoint = ArmConstants.levels[level - 1];

    motorConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pidf(0.6f, 0f, 0f, 0f, ClosedLoopSlot.kSlot0);

    motor.configure(motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    controller.setReference(setPoint, ControlType.kPosition, ClosedLoopSlot.kSlot0, 0);
  }

  // Figure out if it's at a point
  public boolean atLevel(int levlNeeded) {
    boolean atGoodPosition =
        (getEncoder() < (ArmConstants.levels[levlNeeded - 1] + 0.5))
            && (getEncoder() > (ArmConstants.levels[levlNeeded - 1] - 0.5));

    if (atGoodPosition) {
      buttonPressed = false;
      return true;
    }

    return false;
  }
}
