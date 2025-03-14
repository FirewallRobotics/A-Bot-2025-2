package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.VisionSubsystem;

public class AlignWithNearest extends Command {

  public static String name = frc.robot.Constants.VisionSubsystemConstants.limelightName;
  double distance;
  double rotation;

  public static Pose2d[] TagPos = {
    new Pose2d(16.408, 1.048, new Rotation2d(-0.9075712)),
    new Pose2d(16.296, 7.007, new Rotation2d(0.9075712)),
    new Pose2d(11.434, 7.398, new Rotation2d(1.570796)),
    new Pose2d(0, 0, new Rotation2d(0)),
    new Pose2d(0, 0, new Rotation2d(0)),
    new Pose2d(13.787, 2.811, new Rotation2d(2.094395)),
    new Pose2d(14.538, 3.969, new Rotation2d(3.141593)),
    new Pose2d(13.840, 5.217, new Rotation2d(-2.111848)),
    new Pose2d(12.365, 5.165, new Rotation2d(-1.012291)),
    new Pose2d(11.638, 4.007, new Rotation2d(0)),
    new Pose2d(12.390, 2.790, new Rotation2d(1.012291)),
    new Pose2d(1.161, 1.048, new Rotation2d(-2.216568)),
    new Pose2d(1.131, 6.950, new Rotation2d(2.181662)),
    new Pose2d(0, 0, new Rotation2d(0)),
    new Pose2d(0, 0, new Rotation2d(0)),
    new Pose2d(6.364, 0.550, new Rotation2d(-1.570796)),
    new Pose2d(3.390, 2.790, new Rotation2d(1.012291)),
    new Pose2d(2.638, 4.007, new Rotation2d(0)),
    new Pose2d(3.365, 5.165, new Rotation2d(-1.012291)),
    new Pose2d(4.840, 5.217, new Rotation2d(-2.111848)),
    new Pose2d(5.538, 3.969, new Rotation2d(3.141593)),
    new Pose2d(4.787, 2.811, new Rotation2d(2.094395))
  };

  public static Pose2d Tag13 = new Pose2d(1.131, 6.950, new Rotation2d(2.181662));
  public static Pose2d Tag12 = new Pose2d(1.161, 1.048, new Rotation2d(-2.216568));
  public static Pose2d Tag2 = new Pose2d(16.296, 7.007, new Rotation2d(0.9075712));
  public static Pose2d Tag1 = new Pose2d(16.408, 1.048, new Rotation2d(-0.9075712));

  public static Pose2d Tag3 = new Pose2d(11.434, 7.398, new Rotation2d(1.570796));
  public static Pose2d Tag16 = new Pose2d(6.364, 0.550, new Rotation2d(-1.570796));

  public static Pose2d Tag17 = new Pose2d(3.390, 2.790, new Rotation2d(1.012291));
  public static Pose2d Tag18 = new Pose2d(2.638, 4.007, new Rotation2d(0));
  public static Pose2d Tag19 = new Pose2d(3.365, 5.165, new Rotation2d(-1.012291));
  public static Pose2d Tag20 = new Pose2d(4.840, 5.217, new Rotation2d(-2.111848));
  public static Pose2d Tag21 = new Pose2d(5.538, 3.969, new Rotation2d(3.141593));
  public static Pose2d Tag22 = new Pose2d(4.787, 2.811, new Rotation2d(2.094395));

  public static Pose2d Tag6 = new Pose2d(13.787, 2.811, new Rotation2d(2.094395));
  public static Pose2d Tag7 = new Pose2d(14.538, 3.969, new Rotation2d(3.141593));
  public static Pose2d Tag8 = new Pose2d(13.840, 5.217, new Rotation2d(-2.111848));
  public static Pose2d Tag9 = new Pose2d(12.365, 5.165, new Rotation2d(-1.012291));
  public static Pose2d Tag10 = new Pose2d(11.638, 4.007, new Rotation2d(0));
  public static Pose2d Tag11 = new Pose2d(12.390, 2.790, new Rotation2d(1.012291));

  // add vision as a requirement to run
  public AlignWithNearest() {}

  @Override
  public void initialize() {
    /*
    Optional<Alliance> ally = DriverStation.getAlliance();
    if (ally.isPresent()) {
      if (ally.get() == Alliance.Blue) {
        if (VisionSubsystem.DistanceToCoralStation() != -1) {
          if (VisionSubsystem.CanSeeTag(13)) {
            RobotContainer.drivebase.driveToPose(Tag13);
          } else {
            RobotContainer.drivebase.driveToPose(Tag12);
          }
        }
        if (VisionSubsystem.DistanceToReef() != -1) {
          if (VisionSubsystem.CanSeeTag(17)) {
            RobotContainer.drivebase.driveToPose(Tag17);
          } else if (VisionSubsystem.CanSeeTag(18)) {
            RobotContainer.drivebase.driveToPose(Tag18);
          } else if (VisionSubsystem.CanSeeTag(19)) {
            RobotContainer.drivebase.driveToPose(Tag19);
          } else if (VisionSubsystem.CanSeeTag(20)) {
            RobotContainer.drivebase.driveToPose(Tag20);
          } else if (VisionSubsystem.CanSeeTag(21)) {
            RobotContainer.drivebase.driveToPose(Tag21);
          } else {
            RobotContainer.drivebase.driveToPose(Tag22);
          }
        }
        if (VisionSubsystem.DistanceToProcessor() != -1) {
          RobotContainer.drivebase.driveToPose(Tag3);
        }
      }
      if (ally.get() == Alliance.Red) {
        if (VisionSubsystem.DistanceToCoralStation() != -1) {
          if (VisionSubsystem.CanSeeTag(2)) {
            RobotContainer.drivebase.driveToPose(Tag2);
          } else {
            RobotContainer.drivebase.driveToPose(Tag1);
          }
        }
        if (VisionSubsystem.DistanceToReef() != -1) {
          if (VisionSubsystem.CanSeeTag(6)) {
            RobotContainer.drivebase.driveToPose(Tag6);
          } else if (VisionSubsystem.CanSeeTag(7)) {
            RobotContainer.drivebase.driveToPose(Tag7);
          } else if (VisionSubsystem.CanSeeTag(8)) {
            RobotContainer.drivebase.driveToPose(Tag8);
          } else if (VisionSubsystem.CanSeeTag(9)) {
            RobotContainer.drivebase.driveToPose(Tag9);
          } else if (VisionSubsystem.CanSeeTag(10)) {
            RobotContainer.drivebase.driveToPose(Tag10);
          } else {
            RobotContainer.drivebase.driveToPose(Tag11);
          }
        }
        if (VisionSubsystem.DistanceToProcessor() != -1) {
          RobotContainer.drivebase.driveToPose(Tag16);
        }
      }
    }
      */
    for (int i = 0; i < TagPos.length; i++) {
      if (VisionSubsystem.CanSeeTag(i)) {
        RobotContainer.drivebase.driveToPose(TagPos[i]);
      }
    }
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
