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

  // Update PIDF
  public void Periodic() {
    SmartDashboard.putNumber("Elevator-Speed", leftMotor.get());
    SmartDashboard.putNumber("Elevator-EncoderPos", getPositionEncoder());
  }

  public double getSpeed() {
    return leftMotor.get();
  }

  public void setLevel(int level) {
    if (level < 0 || level >= levels.length) {
      System.out.println("Invalid level: " + level);
      return;
    }
    moveToPosition(levels[level]);
    // RobotContainer.coralHoldAngleSubsystem.holdUp(Angles[level]);
  }

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
          getPositionEncoder(), ControlType.kPosition, ClosedLoopSlot.kSlot0, -0.5);
    }
  }

  // -35
  // -16

  public double getLevel() {
    if (Robot.isSimulation()) {
      return levels[(int) SmartDashboard.getNumber("ElevatorPos", 0)];
    }
    return leftMotor.getEncoder().getPosition();
  }

  private void moveToPosition(double position) {
    for (int i = 0; i < levels.length; i++) {
      if (levels[i] == position) {
        SmartDashboard.putNumber("ElevatorPos", i);
      }
    }
    closedLoopController.setReference(position, ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  public void stop() {
    // leftMotor.set(0);
    closedLoopController.setReference(
        getPositionEncoder(), ControlType.kPosition, ClosedLoopSlot.kSlot0, -0.5);
  }

  public double getPositionEncoder() {
    return leftMotor.getEncoder().getPosition();
  }

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
