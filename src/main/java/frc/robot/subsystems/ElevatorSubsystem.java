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
import frc.robot.Constants.ElevatorSubsystemConstants;
import frc.robot.Robot;
import java.util.logging.Level;
import java.util.logging.Logger;

public class ElevatorSubsystem extends SubsystemBase {
  private final SparkFlex leftMotor;
  private final SparkFlex rightMotor;
  private final SparkFlexConfig leftMotorConfig;
  private final SparkFlexConfig rightMotorConfig;

  @SuppressWarnings("unused")
  private RelativeEncoder encoder;

  private SparkClosedLoopController closedLoopController;

  // Elevator levels in encoder ticks
  public static final double[] levels = {0, -18, -22, -46, -50};

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
    leftMotor.configure(
        leftMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    rightMotor.configure(
        rightMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    leftMotorConfig.closedLoop.pidf(0f, 0f, 0f, 0.63f, ClosedLoopSlot.kSlot0);
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
   * @deprecated Due to the PIDF not being tuned this currently doesn't do anything
   */
  public void setLevel(int level) {
    if (level < 0 || level >= levels.length) {
      System.out.println("Invalid level: " + level);
      return;
    }
    moveToPosition(levels[level]);
    // RobotContainer.coralHoldAngleSubsystem.holdUp(Angles[level]);
  }

  /**
   * Sets the speed of the elevator with some safe guards. And holds the position using a blanket
   * 30% feed forward Will not go over 0 on the encoders (elevator position is inverted) or go under
   * -50.1 as reported by the encoder
   */
  public void setSpeed(double speed) {
    if (getPositionEncoder() >= 0 && speed > 0) {
      leftMotor.set(0);
      closedLoopController.setReference(0, ControlType.kPosition, ClosedLoopSlot.kSlot0, -0.3);
    } else if (getPositionEncoder() <= -50.1 && speed < 0) {
      leftMotor.set(0);
      closedLoopController.setReference(0, ControlType.kPosition, ClosedLoopSlot.kSlot0, -0.3);
    } else {
      leftMotor.set(speed);
    }
  }

  /** Goes to the level 3 position using if statements */
  public void goToL3() {
    double setPoint = -39;
    if (setPoint + 1 >= getPositionEncoder()) {
      leftMotor.set(0.15);
      Logger.getGlobal().log(Level.INFO, "Going Down");
    }
    if (setPoint - 1 <= getPositionEncoder()) {
      leftMotor.set(-0.3);
      Logger.getGlobal().log(Level.INFO, "Going Up");
    }
    if (setPoint - 2 >= getPositionEncoder() && setPoint + 2 <= getPositionEncoder()) {
      Logger.getGlobal().log(Level.INFO, "Found L3");
      leftMotor.set(0);
      closedLoopController.setReference(
          getPositionEncoder(), ControlType.kPosition, ClosedLoopSlot.kSlot0, -0.3);
    }
  }

  // -35
  // -16

  /**
   * Move to a position for the elevator to move to using PIDF. Will also update the simulation of
   * the elevator
   *
   * @deprecated As of yet the PIDF has not been tuned so this does nothing
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
    // leftMotor.set(0);
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
