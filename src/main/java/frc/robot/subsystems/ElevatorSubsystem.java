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
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorSubsystemConstants;
import frc.robot.Robot;
import java.util.logging.Level;
import java.util.logging.Logger;

public class ElevatorSubsystem extends SubsystemBase {
  private final SparkFlex leftMotor;
  private final SparkFlex rightMotor;
  private final SparkFlexConfig leftMotorConfig;
  private final SparkFlexConfig rightMotorConfig;

  public boolean levelThere;

  private TrapezoidProfile.State state;

  private boolean gottenOgPos;

  private RelativeEncoder encoder;

  private SparkClosedLoopController closedLoopController;

  // private ArmFeedforward armFeedforward =
  //     new ArmFeedforward(
  //         ElevatorSubsystemConstants.kSVolts,
  //         ElevatorSubsystemConstants.kGVolts,
  //         ElevatorSubsystemConstants.kVVoltSecondPerRad,
  //         ElevatorSubsystemConstants.kAVoltSecondSquaredPerRad);
  // private ElevatorFeedforward feedforward =
  //     new ElevatorFeedforward(
  //         ElevatorSubsystemConstants.kSVolts,
  //         ElevatorSubsystemConstants.kGVolts,
  //         ElevatorSubsystemConstants.kVVoltSecondPerRad,
  //         ElevatorSubsystemConstants.kAVoltSecondSquaredPerRad);

  // Elevator levels in encoder ticks
  // 0 - Intake position
  // 1 - level 2
  // 2 - level 3
  public static final double[] levels = {0, -15.056, -31.401};

  // public static final double[] Angles = {0, 0, 0, 0, 0};

  public ElevatorSubsystem() {

    leftMotor =
        new SparkFlex(
            ElevatorSubsystemConstants.ELEVATOR_LEFT_MOTOR_ID,
            MotorType.kBrushless); // Assign motor controller port
    rightMotor =
        new SparkFlex(
            ElevatorSubsystemConstants.ELEVATOR_RIGHT_MOTOR_ID,
            MotorType.kBrushless); // Assign motor controller port

    closedLoopController = leftMotor.getClosedLoopController();
    encoder = leftMotor.getEncoder();
    leftMotorConfig = new SparkFlexConfig();
    rightMotorConfig = new SparkFlexConfig();
    leftMotorConfig.encoder.positionConversionFactor(1);
    leftMotorConfig.encoder.velocityConversionFactor(1);
    leftMotorConfig.smartCurrentLimit(50);
    rightMotorConfig.smartCurrentLimit(50);
    leftMotorConfig.idleMode(IdleMode.kBrake);
    rightMotorConfig.idleMode(IdleMode.kBrake);
    rightMotorConfig.inverted(false);
    rightMotorConfig.follow(leftMotor, true);

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

  /**
   * Get the current speed that the motors are moving at
   *
   * @apiNote The elevator is inverted (-1 goes up and 1 goes down)
   * @return Left motor speed(both motors are mirrored)
   */
  public double getSpeed() {
    return leftMotor.get();
  }

  /**
   * Set the level that the elevator should move too
   *
   * @apiNote Currently uses {@link #goToLevelNoPID(double)} which will be shakey
   */
  public void setLevel(int level) {
    if (level < 0 || level >= levels.length) {
      System.out.println("Invalid level: " + level);
      return;
    }
    goToLevelNoPID(levels[level]);
    // RobotContainer.coralHoldAngleSubsystem.holdUp(Angles[level]);
  }

  /**
   * Sets the speed of the elevator with some safe guards. And holds the position using a blanket
   * 30% feed forward Will not go over 0 on the encoders (elevator position is inverted) or go under
   * -50.1 as reported by the encoder
   */
  public void setSpeed(double speed) {
    leftMotor.set(speed);
    // if (getPositionEncoder() >= 0 && speed > 0) {
    //   leftMotor.set(0);
    //   //closedLoopController.setReference(0, ControlType.kPosition, ClosedLoopSlot.kSlot0, -0.3);
    // } else if (getPositionEncoder() <= -50.1 && speed < 0) {
    //   leftMotor.set(0);
    //   //closedLoopController.setReference(0, ControlType.kPosition, ClosedLoopSlot.kSlot0, -0.3);
    // } else {
    //   leftMotor.set(speed);
    // }
  }

  public double finalLevelPos(int levelWanted) {
    return levels[levelWanted - 1];
  }

  public void goToCoralLevel(int levelWanted) {

    double setPoint = levels[levelWanted - 1];

    leftMotorConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pidf(0.000001f, 0f, 0f, 0.3f, ClosedLoopSlot.kSlot0);

    leftMotor.configure(
        leftMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    rightMotor.configure(
        rightMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    closedLoopController.setReference(setPoint, ControlType.kPosition, ClosedLoopSlot.kSlot0, -0.3);

    // double setPoint = levels[levelWanted - 1];
    // // state = new State(setPoint, 0);

    // // double ff = armFeedforward.calculate(state.position * 2 * Math.PI, state.velocity);
    // // closedLoopController.setReference(0, ControlType.kPosition, ClosedLoopSlot.kSlot0, ff);

    // // double setPoint = levels[levelWanted - 1];
    // // state = new State(setPoint, 0);
    // // double ff = feedforward.calculate(state.position, state.velocity);
    // // closedLoopController.setReference(0, ControlType.kPosition, ClosedLoopSlot.kSlot0, ff);

    // // while (!((setPoint - 1 >= getPositionEncoder()) && (setPoint + 3 <=
    // getPositionEncoder()))) {
    // //   if (setPoint + 3 >= getPositionEncoder()) {
    // //     leftMotor.set(0.15);
    // //     //Logger.getGlobal().log(Level.INFO, "Going Down");
    // //   } else if (setPoint - 1 <= getPositionEncoder()) {
    // //     leftMotor.set(-0.3);
    // //     //Logger.getGlobal().log(Level.INFO, "Going Up");
    // //   }

    // //   // if (setPoint - 2 >= getPositionEncoder() && setPoint + 2 <= getPositionEncoder()) {
    // //   //   //Logger.getGlobal().log(Level.INFO, "Found L3");
    // //   //   leftMotor.set(0);
    // //   //   closedLoopController.setReference(
    // //   //   getPositionEncoder(), ControlType.kPosition, ClosedLoopSlot.kSlot0, -0.5);
    // //   // }

    // // }
    // // closedLoopController.setReference(
    // // getPositionEncoder(), ControlType.kPosition, ClosedLoopSlot.kSlot0, -0.5);
    // if (setPoint + 4 >= getPositionEncoder()) {
    //     leftMotor.set(0.15);
    //       //Logger.getGlobal().log(Level.INFO, "Going Down");
    // } else if (setPoint - 1 <= getPositionEncoder()) {
    //       leftMotor.set(-0.3);
    //      //Logger.getGlobal().log(Level.INFO, "Going Up");
    //  }

    // if (setPoint - 1 >= getPositionEncoder() && setPoint + 4 <= getPositionEncoder()) {
    //   Logger.getGlobal().log(Level.INFO, "Found L3");
    //   leftMotor.set(0);
    //   closedLoopController.setReference(
    //       getPositionEncoder(), ControlType.kPosition, ClosedLoopSlot.kSlot0, -0.5);
    // }

    // double ff = feedforward.calculate(setpoint.position * 2 * Math.PI, setpoint.velocity);
    // closedLoopController.setReference(0, ControlType.kPosition, ClosedLoopSlot.kSlot0, ff);
  }

  public boolean atLevel(int levlNeeded) {
    Logger.getGlobal().log(Level.INFO, "looking for level");
    Logger.getGlobal().log(Level.INFO, "encoder value " + getPositionEncoder());
    Logger.getGlobal().log(Level.INFO, "target value " + levels[levlNeeded - 1]);

    return getPositionEncoder() < levels[levlNeeded - 1];
  }

  public void getStartPos() {
    double ogOffSet = encoder.getPosition();
    levels[1] = levels[1] + ogOffSet;
    levels[2] = levels[2] + ogOffSet;

  }

  // -35
  // -16


  public double getLevel() {
    if (Robot.isSimulation()) {
      return levels[(int) SmartDashboard.getNumber("ElevatorPos", 0)];
    }
    return leftMotor.getEncoder().getPosition();
  }

  public boolean getOgPOSgotten() {
    return gottenOgPos;
  }

  public void setOgPOSgotten() {
    gottenOgPos = true;
  }


  /**
   * Goes to a setpoint using if statements. (No PIDF) But will hold using PIDF
   *
   * @param setPoint relative encoder position desired
   */
  public void goToLevelNoPID(double setPoint) {
    // double setPoint = -39;
    if (setPoint + 1 >= getPositionEncoder()) {
      leftMotor.set(0.15);
      Logger.getGlobal().log(Level.INFO, "Going Down");
    }
    if (setPoint - 1 <= getPositionEncoder()) {
      leftMotor.set(-0.3);
      Logger.getGlobal().log(Level.INFO, "Going Up");
    }
    if (setPoint - 2 >= getPositionEncoder() && setPoint + 2 <= getPositionEncoder()) {
      Logger.getGlobal().log(Level.INFO, "Found Level");
      leftMotor.set(0);
      closedLoopController.setReference(
          getPositionEncoder(), ControlType.kPosition, ClosedLoopSlot.kSlot0, -0.3);
    }
  }
  
  /**
   * Move to a position for the elevator to move to using PIDF. Will also update the simulation of
   * the elevator
   *
   */
  private void moveToPosition(double position) {
    for (int i = 0; i < levels.length; i++) {
      if (levels[i] == position) {
        SmartDashboard.putNumber("ElevatorPos", i);
      }
    }
    closedLoopController.setReference(position, ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  /** Stop the elevator from moving and hold the position with a flat 30% feed forward */
  public void stop() {
    leftMotor.set(0);

    leftMotorConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pidf(0f, 0f, 0f, 0, ClosedLoopSlot.kSlot0);

    leftMotor.configure(
        leftMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    rightMotor.configure(
        rightMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    closedLoopController.setReference(
        getPositionEncoder(), ControlType.kPosition, ClosedLoopSlot.kSlot0, -0.3);
  }

  /**
   * Get the position of the elevator encoder
   *
   * @return Position of the left motor encoder(the elevator is mirrored so it doesn't matter)
   */
  public double getPositionEncoder() {
    return leftMotor.getEncoder().getPosition();
  }

  /**
   * If the elevator is finished moving to a position If we are in the sim blanket return 0 as we
   * cannot move a mechanism
   */
  public boolean isFinished(int position) {
    if (Robot.isSimulation()) {
      return true;
    }
    if (levels[position] - (leftMotor.getEncoder().getPosition()) == 0) {
      return true;
    } else {
      return false;
    }
  }
}
