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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorSubsystemConstants;

public class ElevatorSubsystem extends SubsystemBase {
  private final SparkFlex leftMotor;
  private final SparkFlex rightMotor;
  private final SparkFlexConfig leftMotorConfig;
  private final SparkFlexConfig rightMotorConfig;
  private RelativeEncoder encoder;
  private SparkClosedLoopController closedLoopController;

  // Elevator levels in encoder ticks
  private final double[] levels = {0, 1000, 2000, 3000, 4000};

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
    rightMotorConfig.inverted(true);
    rightMotorConfig.follow(leftMotor);
    leftMotor.configure(
        leftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightMotor.configure(
        rightMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setLevel(int level) {
    if (level < 0 || level >= levels.length) {
      System.out.println("Invalid level: " + level);
      return;
    }
    moveToPosition(levels[level]);
  }

  private void moveToPosition(double position) {
    closedLoopController.setReference(position, ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  public void stop() {
    leftMotor.set(0);
  }

  public boolean isFinished(int position) {
    if (levels[position] - (leftMotor.getEncoder().getPosition()) == 0) {
      return true;
    } else {
      return false;
    }
  }
}
