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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorSubsystemConstants;

public class ElevatorCoralSubsystem extends SubsystemBase {
  // The two motors
  private final SparkFlex leftMotor;
  private final SparkFlex rightMotor;
  private final SparkFlexConfig leftMotorConfig;
  private final SparkFlexConfig rightMotorConfig;

  private RelativeEncoder encoder;

  // Controller
  private SparkClosedLoopController closedLoopController;

  // Constructor
  public ElevatorCoralSubsystem() {
    // Set up motors
    leftMotor =
        new SparkFlex(
            ElevatorSubsystemConstants.ELEVATOR_LEFT_MOTOR_ID,
            MotorType.kBrushless); // Assign motor controller port
    rightMotor =
        new SparkFlex(
            ElevatorSubsystemConstants.ELEVATOR_RIGHT_MOTOR_ID,
            MotorType.kBrushless); // Assign motor controller port

    // Controller
    closedLoopController = leftMotor.getClosedLoopController();

    encoder = leftMotor.getEncoder();

    // All configuration stuff
    leftMotorConfig = new SparkFlexConfig();
    rightMotorConfig = new SparkFlexConfig();
    leftMotorConfig.encoder.positionConversionFactor(1);
    leftMotorConfig.encoder.velocityConversionFactor(1);
    // Current limiter
    leftMotorConfig.smartCurrentLimit(38);
    rightMotorConfig.smartCurrentLimit(38);
    leftMotorConfig.idleMode(IdleMode.kBrake);
    rightMotorConfig.idleMode(IdleMode.kBrake);
    rightMotorConfig.inverted(false);
    rightMotorConfig.follow(leftMotor, true);

    // PID manual
    leftMotorConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pidf(0f, 0f, 0f, 0.63f, ClosedLoopSlot.kSlot0);

    leftMotor.configure(
        leftMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    rightMotor.configure(
        rightMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void Periodic() {
    SmartDashboard.putNumber("Elevator-Speed", leftMotor.get());
    SmartDashboard.putNumber("Elevator-EncoderPos", getPositionEncoder());
  }

  // Get position encoder
  public double getPositionEncoder() {
    return leftMotor.getEncoder().getPosition();
  }

  // Elevator going up
  public void ElevatorUp() {
    leftMotor.set(-(ElevatorSubsystemConstants.elevatorUpSpeed));
  }

  // Elevator going down
  public void ElevatorDown() {
    leftMotor.set(ElevatorSubsystemConstants.elevatorDownSpeed);
  }

  // Elevator stop
  public void ElevatorStop() {

    leftMotor.set(0);

    PIDSelect(ElevatorSubsystemConstants.elevatorPNumbers[2]);

    closedLoopController.setReference(
        getPositionEncoder(), ControlType.kPosition, ClosedLoopSlot.kSlot0, -0.3);
  }

  // Go up to a coral level
  public void ToCoralLevel(int level) {

    double setPoint = ElevatorSubsystemConstants.elevatorLevels[level - 1];

    if (getPositionEncoder() > setPoint) {
      PIDSelect(ElevatorSubsystemConstants.elevatorPNumbers[0]);
    } else if (getPositionEncoder() < setPoint) {
      PIDSelect(ElevatorSubsystemConstants.elevatorPNumbers[1], 0.1f);
    }

    closedLoopController.setReference(setPoint, ControlType.kPosition, ClosedLoopSlot.kSlot0, 0);
  }

  // Check if we are at a coral level
  public boolean atLevel(int levlNeeded) {
    // boolean atGoodPosition =
    //     (getPositionEncoder() == (ElevatorSubsystemConstants.elevatorLevels[levlNeeded - 1]));
    boolean atGoodPosition =
        (getPositionEncoder() < (ElevatorSubsystemConstants.elevatorLevels[levlNeeded - 1] + 0.1))
            && (getPositionEncoder()
                > (ElevatorSubsystemConstants.elevatorLevels[levlNeeded - 1] - 0.1));

    if (atGoodPosition) {
      return true;
    }

    return false;
  }

  // Get the different PIDs needed for UP, DOWN, and STOP
  public void PIDSelect(float p) {

    leftMotorConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pidf(p, 0f, 0f, 0f, ClosedLoopSlot.kSlot0);

    leftMotor.configure(
        leftMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    rightMotor.configure(
        rightMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  // Get the different PIDs needed for UP, DOWN, and STOP
  public void PIDSelect(float p, float d) {

    leftMotorConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pidf(p, 0f, d, 0f, ClosedLoopSlot.kSlot0);

    leftMotor.configure(
        leftMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    rightMotor.configure(
        rightMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }
}
