// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class AlgaeSubsystem extends SubsystemBase {
private final SparkMax m_highAlgaeMotor, m_lowAlgaeMotor;
  /** Creates a new AlgaeSubsystem. */
  public AlgaeSubsystem() {
    // motors
    m_highAlgaeMotor = new SparkMax(AlgaeConstants.kHighAlgaeCANId, MotorType.kBrushless);
    m_lowAlgaeMotor = new SparkMax(AlgaeConstants.kLowAlgaeCANId, MotorType.kBrushless);

    m_highAlgaeMotor.configure(Configs.AlgaeSubsystem.highAlgaeMotorConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    m_lowAlgaeMotor.configure(Configs.AlgaeSubsystem.lowAlgaeMotorConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
     //SmartDashboard.putNumber("Coral_Left RPM", m_leftCoralEncoder.getVelocity());
   // SmartDashboard.putNumber("Coral_Right RPM", m_rightCoralEncoder.getVelocity());
  }

  public void ShootAlgaeOut(){
    m_highAlgaeMotor.set(AlgaeConstants.kAlgaeSpeedOut);
    m_lowAlgaeMotor.set(-AlgaeConstants.kAlgaeSpeedOut);
  }

  public void FeedAlgaeIn(){
    m_highAlgaeMotor.set(-AlgaeConstants.kAlgaeSpeedIn);
    m_lowAlgaeMotor.set(AlgaeConstants.kAlgaeSpeedIn);
  }

  public void AlgaeStop() {
    m_highAlgaeMotor.set(0);
    m_lowAlgaeMotor.set(0);
  }

  // same as ShootCoralOut() except is a Command class
  public Command ShootAlgaeOutCmd() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
                m_highAlgaeMotor.set(AlgaeConstants.kAlgaeSpeedOut);
                m_lowAlgaeMotor.set(-AlgaeConstants.kAlgaeSpeedOut);
              }
    );
  }
  // same as shooterFeedNoteIn() except it is a Command class
   public Command FeedAlgaeInCmd() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
                m_highAlgaeMotor.set(-AlgaeConstants.kAlgaeSpeedIn);
                m_lowAlgaeMotor.set(AlgaeConstants.kAlgaeSpeedIn);
              }
    );
  }

  // same as shooterStop() except it is a Command class
  public Command AlgaeStopCmd() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
                m_highAlgaeMotor.set(0);
                m_lowAlgaeMotor.set(0);
              }
    );
  }
  }