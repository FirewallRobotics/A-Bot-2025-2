// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
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

  public static class AlgaeSubsystemConstants {
    public static final int ALGAE_MOTOR_ID = 15;
  }

  public static class ElevatorSubsystemConstants {
    public static final int ELEVATOR_LEFT_MOTOR_ID = 11;
    public static final int ELEVATOR_RIGHT_MOTOR_ID = 12;
  }

  public static class ClimberSubsystemConstants {
    public static final int CLIMBER_LEFT_MOTOR_ID = 13;
    public static final int CLIMBER_RIGHT_MOTOR_ID = 14;
  }

  public static class VisionSubsystemConstants {
    public static final String limelightName = "limelight-cyclops";
  }

  public static class OperatorConstants {

    // Joystick Deadband
    public static final double DEADBAND = 0.1;
    public static final double LEFT_X_DEADBAND = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT = 6;
    public static final double LEFT_Y_DEconfigureBindingsADBAND = 0;
  }

  public static class ArmConstants {

    public static final double kSVolts = 0.11356;
    public static final double kGVolts = 0.29175;
    public static final double kVVoltSecondPerRad = 1.5928;
    public static final double kAVoltSecondSquaredPerRad = 0.030171;
  }
}
