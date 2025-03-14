package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {
  /** Creates a new Climbers. */
  SparkMax m_higherClimber;

  SparkMax m_lowerClimber;

  RelativeEncoder m_leftEncoder;
  RelativeEncoder m_rightEncoder;

  final SparkMaxConfig m_config = new SparkMaxConfig();

  static final double EXTENSION_SPEED = -0.50;
  static final double RETRACTION_SPEED = 0.50;

  public ClimberSubsystem() {
    m_higherClimber =
        new SparkMax(
            Constants.ClimberSubsystemConstants.CLIMBER_LEFT_MOTOR_ID, MotorType.kBrushless);
    m_lowerClimber =
        new SparkMax(
            Constants.ClimberSubsystemConstants.CLIMBER_RIGHT_MOTOR_ID, MotorType.kBrushless);

    m_leftEncoder = m_higherClimber.getEncoder();
    m_rightEncoder = m_lowerClimber.getEncoder();

    m_config.idleMode(SparkBaseConfig.IdleMode.kBrake);

    m_higherClimber.configure(
        m_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_lowerClimber.configure(
        m_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void StartExtending() {
    m_higherClimber.set(EXTENSION_SPEED);
  }

  public void StartRetracting() {
    m_lowerClimber.set(RETRACTION_SPEED);
  }

  public void stop() {
    m_higherClimber.stopMotor();
    m_lowerClimber.stopMotor();
  }

  public double getEncoder() {
    return m_leftEncoder.getPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }
}
