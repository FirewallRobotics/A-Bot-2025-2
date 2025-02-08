// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.LinearVelocityUnit;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final int kMotorPort = 0;
  public static final int kEncoderAChannel = 0;
  public static final int kEncoderBChannel = 1;
  public static final int kJoystickPort = 0;

  public static final Mechanism2d sideRobotView = new Mechanism2d(36, 72);
  public static final MechanismRoot2d kElevatorCarriage;
  // public static final MechanismLigament2d kArmMech;
  public static final MechanismLigament2d kElevatorTower;

  static {
    kElevatorCarriage =
        Constants.sideRobotView.getRoot(
            "ElevatorCarriage", 10.5, ElevatorSubsystemConstants.kStartingHeightSim.in(Meters));
    /*  kArmMech = kElevatorCarriage.append(
    new MechanismLigament2d(
        "Arm",
        ArmConstants.kArmLength,
        ArmConstants.kArmStartingAngle.in(Degrees),
        6,
        new Color8Bit(Color.kYellow))); */
    kElevatorTower =
        kElevatorCarriage.append(
            new MechanismLigament2d(
                "Elevator",
                ElevatorSubsystemConstants.kStartingHeightSim.in(Meters),
                -90,
                6,
                new Color8Bit(Color.kRed)));
  }

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS =
      new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME = 0.13; // s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED = Units.feetToMeters(14.5);

  // Maximum speed of the robot in meters per second, used to limit acceleration.

  //  public static final class AutonConstants
  //  {
  //
  //    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
  //    public static final PIDConstants ANGLE_PID       = new PIDConstants(0.4, 0, 0.01);
  //  }

  public static final class DrivebaseConstants {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class CoralHoldAngleSubsystemConstants {
    public static final int CORAL_HOLD_ANGLE_MOTOR_ID = 9;
  }

  public static class CoralHoldSubsystemConstants {
    public static final int CORAL_HOLD_MOTOR_ID = 10;
  }

  public static class ElevatorSubsystemConstants {
    public static final int ELEVATOR_LEFT_MOTOR_ID = 11;
    public static final int ELEVATOR_RIGHT_MOTOR_ID = 12;
    public static final double kElevatorKp = 26.722;
    public static final double kElevatorKi = 0;
    public static final double kElevatorKd = 1.6047;

    public static final double kElevatorkS = 0.01964; // volts (V)
    public static final double kElevatorkV = 3.894; // volt per velocity (V/(m/s))
    public static final double kElevatorkA = 0.173; // volt per acceleration (V/(m/s²))
    public static final double kElevatorkG = 0.91274; // volts (V)

    public static final double kElevatorGearing = 10.0;
    public static final double kElevatorDrumRadius = Units.inchesToMeters(2.0);
    public static final double kCarriageMass = 4.0; // kg

    // Encoder is reset to measure 0 at the bottom, so minimum height is 0.
    public static final Distance kLaserCANOffset = Inches.of(3);
    public static final Distance kStartingHeightSim = Meters.of(0);
    public static final Distance kMinElevatorHeight = Meters.of(0.0);
    public static final Distance kMaxElevatorHeight = Meters.of(10.25);
    private static final LinearVelocityUnit MetersPerSecond = null;

    public static double kElevatorRampRate = 0.1;
    public static int kElevatorCurrentLimit = 40;
    public static double kMaxVelocity = Meters.of(4).per(Second).in(MetersPerSecond);
    public static double kMaxAcceleration =
        Meters.of(8).per(Second).per(Second).in(MetersPerSecondPerSecond);
  }

  public static class FlexAutoSubsystemConstants {}

  public static class OperatorConstants {

    // Joystick Deadband
    public static final double LEFT_X_DEADBAND = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT = 6;
    public static final double LEFT_Y_DEconfigureBindingsADBAND = 0;
  }
}
