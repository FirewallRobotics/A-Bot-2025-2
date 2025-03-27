// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import java.util.ArrayList;
import java.util.Arrays;
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

  private static double distanceAway = 1;

  public static Pose2d[] reefFaces = {
    new Pose2d(
        Units.inchesToMeters(144.003), Units.inchesToMeters(158.500), Rotation2d.fromDegrees(180)),
    new Pose2d(
        Units.inchesToMeters(160.373), Units.inchesToMeters(186.857), Rotation2d.fromDegrees(120)),
    new Pose2d(
        Units.inchesToMeters(193.116), Units.inchesToMeters(186.858), Rotation2d.fromDegrees(60)),
    new Pose2d(
        Units.inchesToMeters(209.489), Units.inchesToMeters(158.502), Rotation2d.fromDegrees(0)),
    new Pose2d(
        Units.inchesToMeters(193.118), Units.inchesToMeters(130.145), Rotation2d.fromDegrees(-60)),
    new Pose2d(
        Units.inchesToMeters(160.375), Units.inchesToMeters(130.144), Rotation2d.fromDegrees(-120)),
    new Pose2d(
        Units.inchesToMeters(68) + 0.5,
        Units.inchesToMeters(71) + 0.5,
        Rotation2d.fromDegrees(-126)), // this is Right source, angle could be inaccurate
    new Pose2d(
        Units.inchesToMeters(67.5) + 0.5,
        Units.inchesToMeters(244.5) - 0.5,
        Rotation2d.fromDegrees(126)), // this is left source, angle could be inaccurate
    new Pose2d(
        Units.inchesToMeters(240),
        Units.inchesToMeters(55) + 0.75,
        Rotation2d.fromDegrees(-90)) // this is processer, angle could be inaccurate
  };

  public enum ReefScorePositions {
    FRONT(
        new Pose2d(
            Math.cos(reefFaces[0].getRotation().getRadians()) * distanceAway
                + reefFaces[0].getTranslation().getX(),
            Math.sin(reefFaces[0].getRotation().getRadians()) * distanceAway
                + reefFaces[0].getTranslation().getY(),
            reefFaces[0].getRotation()),
        18),
    FRONTLEFT(
        new Pose2d(
            Math.cos(reefFaces[1].getRotation().getRadians()) * distanceAway
                + reefFaces[1].getTranslation().getX(),
            Math.sin(reefFaces[1].getRotation().getRadians()) * distanceAway
                + reefFaces[1].getTranslation().getY(),
            reefFaces[1].getRotation()),
        19),

    BACKLEFT(
        new Pose2d(
            Math.cos(reefFaces[2].getRotation().getRadians()) * distanceAway
                + reefFaces[2].getTranslation().getX(),
            Math.sin(reefFaces[2].getRotation().getRadians()) * distanceAway
                + reefFaces[2].getTranslation().getY(),
            reefFaces[2].getRotation()),
        20),
    BACK(
        new Pose2d(
            Math.cos(reefFaces[3].getRotation().getRadians()) * distanceAway
                + reefFaces[3].getTranslation().getX(),
            Math.sin(reefFaces[3].getRotation().getRadians()) * distanceAway
                + reefFaces[3].getTranslation().getY(),
            reefFaces[3].getRotation()),
        21),
    BACKRIGHT(
        new Pose2d(
            Math.cos(reefFaces[4].getRotation().getRadians()) * distanceAway
                + reefFaces[4].getTranslation().getX(),
            Math.sin(reefFaces[4].getRotation().getRadians()) * distanceAway
                + reefFaces[4].getTranslation().getY(),
            reefFaces[4].getRotation()),
        22),
    FRONTRIGHT(
        new Pose2d(
            Math.cos(reefFaces[5].getRotation().getRadians()) * distanceAway
                + reefFaces[5].getTranslation().getX(),
            Math.sin(reefFaces[5].getRotation().getRadians()) * distanceAway
                + reefFaces[5].getTranslation().getY(),
            reefFaces[5].getRotation()),
        17),
    RIGHTSOURCE(
        new Pose2d(
            Math.cos(reefFaces[6].getRotation().getRadians()) * distanceAway
                + reefFaces[6].getTranslation().getX(),
            Math.sin(reefFaces[6].getRotation().getRadians()) * distanceAway
                + reefFaces[6].getTranslation().getY(),
            reefFaces[6].getRotation()),
        12),
    LEFTSOURCE(
        new Pose2d(
            Math.cos(reefFaces[7].getRotation().getRadians()) * distanceAway
                + reefFaces[7].getTranslation().getX(),
            Math.sin(reefFaces[7].getRotation().getRadians()) * distanceAway
                + reefFaces[7].getTranslation().getY(),
            reefFaces[7].getRotation()),
        13),
    PROCESSER(
        new Pose2d(
            Math.cos(reefFaces[8].getRotation().getRadians()) * distanceAway
                + reefFaces[8].getTranslation().getX(),
            Math.sin(reefFaces[8].getRotation().getRadians()) * distanceAway
                + reefFaces[8].getTranslation().getY(),
            reefFaces[8].getRotation()),
        16);

    public Pose2d scorePosition;
    public int aprilTagID;

    private ReefScorePositions(Pose2d pose, int aprilTagID) {
      this.scorePosition = pose;
      this.aprilTagID = aprilTagID;
    }
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

  public static class AlgaeSubsystemConstants {
    public static final int ALGAE_MOTOR_ID = 15;
  }

  public static class ElevatorSubsystemConstants {
    public static final int ELEVATOR_LEFT_MOTOR_ID = 11;
    public static final int ELEVATOR_RIGHT_MOTOR_ID = 12;

    public static final double kSVolts = 0.11356;
    public static final double kGVolts = 0.29175;
    public static final double kVVoltSecondPerRad = 1.5928;
    public static final double kAVoltSecondSquaredPerRad = 0.030171;
  }

  public static class ClimberSubsystemConstants {
    public static final int CLIMBER_LEFT_MOTOR_ID = 13;
    public static final int CLIMBER_RIGHT_MOTOR_ID = 14;
  }

  public static class VisionSubsystemConstants {
    public static final String limelightName = "limelight-cyclops";
  }

  public static class VisionConstants {

    // Cam names set using Arducam serial number utility. On DS PC.
    public static final String upperCameraName = "Bcam9782";
    public static final String lowerCameraName = "Acam9782";

    public static final Transform3d upperCameraToRobot =
        new Transform3d(
            new Translation3d(
                Units.inchesToMeters(6.75),
                Units.inchesToMeters(8.25),
                Units.inchesToMeters(38.875)),
            new Rotation3d(0, Units.degreesToRadians(-24), Units.degreesToRadians(0)));

    public static final Transform3d lowerCameraToRobot =
        new Transform3d(
            new Translation3d(
                Units.inchesToMeters(14), Units.inchesToMeters(0), Units.inchesToMeters(10.5)),
            new Rotation3d(0, 0, 0));

    /*
     * Tags of each reef side.
     * Starts at the side closest to the driver station
     * Goes clockwise (relative to driver station)
     */
    public static final ArrayList<Integer> redReefTags =
        new ArrayList<>(Arrays.asList(7, 6, 11, 10, 9, 8));
    public static final ArrayList<Integer> blueReefTags =
        new ArrayList<>(Arrays.asList(18, 19, 20, 21, 22, 17));
    public static ArrayList<Integer> reefTags = new ArrayList<>();

    // Tags of human player stations
    // Starts at left human player station from driver POV
    public static final ArrayList<Integer> redHPTags = new ArrayList<>(Arrays.asList(1, 2));
    public static final ArrayList<Integer> blueHPTags = new ArrayList<>(Arrays.asList(13, 12));
    public static ArrayList<Integer> HPTags = new ArrayList<>();

    // Standard deviations below are from Team Spectrum 3847s X-Ray robot

    /**
     * Standard deviations of model states. Increase these numbers to trust your model's state
     * estimates less. This matrix is in the form [x, y, theta], with units in meters and radians,
     * then meters.
     */
    public static final Matrix<N3, N1> stateStdDevs = VecBuilder.fill(0.1, 0.1, 10);

    /**
     * Standard deviations of the vision measurements. Increase these numbers to trust global
     * measurements from vision less. This matrix is in the form [x, y, theta], with units in meters
     * and radians.
     */
    public static final Matrix<N3, N1> measurementStdDevs = VecBuilder.fill(5, 5, 500);

    // The standard deviations of our vision estimated poses, which affect correction rate
    public static final Matrix<N3, N1> singleTagStdDevs = VecBuilder.fill(2, 2, 8);
    public static final Matrix<N3, N1> multiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);

    public static final AprilTagFieldLayout aprilTagFieldLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);

    public static final Distance reefAlignFrontOffset = Inches.of(18);
    public static final Distance reefAlignLeftStrafeOffset = Inches.of(-4);
    public static final Distance reefAlignRightStrafeOffset = Inches.of(4);

    public static final double reefXOffsetInches = 15;
    public static final double reefYRightOffsetInches = 5;
    public static final double reefYLeftOffsetInches = -7;

    public static final double hpXOffsetInches = 15;
    public static final double hpYOffsetInches = 0;
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
