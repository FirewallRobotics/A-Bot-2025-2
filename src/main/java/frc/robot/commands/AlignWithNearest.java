package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
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
  public void execute() {

    // if we are close enough to the reef
    if (VisionSubsystem.DistanceToReef() != -1
        || VisionSubsystem.DistanceToProcessor() != -1
        || VisionSubsystem.DistanceToCoralStation() != -1) {

      LimelightHelpers.setPipelineIndex(name, 1);
      // if we are to much too the right move left
      if (LimelightHelpers.getTY(name) > 1) {
        RobotContainer.drivebase.drive(
            new Translation2d(0, SmartDashboard.getNumber("AutoMoveSpeed", 5)), 0, false);
      }
      // if we are too much to the left move right
      else if (LimelightHelpers.getTY(name) < -1) {
        RobotContainer.drivebase.drive(
            new Translation2d(0, -SmartDashboard.getNumber("AutoMoveSpeed", 5)), 0, false);
      }
      // if we are too far from the tag move forward
      if (VisionSubsystem.DistanceToReef() > 0
          || VisionSubsystem.DistanceToProcessor() > 0
          || VisionSubsystem.DistanceToCoralStation() > 0) {
        LimelightHelpers.setPipelineIndex(name, 0);
        if (VisionSubsystem.DistanceToReef() != -1) {
          distance = VisionSubsystem.DistanceToReef();
          if (VisionSubsystem.getReefLocationPose3d().getRotation().getY() > 0.5) {
            rotation = -SmartDashboard.getNumber("AutoScanSpeed", 1.0);
            RobotContainer.drivebase.drive(new Translation2d(distance, 0), rotation, false);
          } else if (VisionSubsystem.getReefLocationPose3d().getRotation().getY() < -0.5) {
            rotation = SmartDashboard.getNumber("AutoScanSpeed", 1.0);
            RobotContainer.drivebase.drive(new Translation2d(distance, 0), rotation, false);
          } else {
            RobotContainer.drivebase.drive(new Translation2d(distance, 0), 0, false);
          }
        } else if (VisionSubsystem.DistanceToProcessor() != -1) {
          distance = VisionSubsystem.DistanceToProcessor();
          if (VisionSubsystem.getProcessorLocationPose3d().getRotation().getY() > 0.5) {
            rotation = -SmartDashboard.getNumber("AutoScanSpeed", 1.0);
            RobotContainer.drivebase.drive(new Translation2d(distance, 0), rotation, false);
          } else if (VisionSubsystem.getProcessorLocationPose3d().getRotation().getY() < -0.5) {
            rotation = SmartDashboard.getNumber("AutoScanSpeed", 1.0);
            RobotContainer.drivebase.drive(new Translation2d(distance, 0), rotation, false);
          } else {
            RobotContainer.drivebase.drive(new Translation2d(distance, 0), 0, false);
          }
        } else if (VisionSubsystem.DistanceToCoralStation() != -1) {
          distance = VisionSubsystem.DistanceToCoralStation();
          if (VisionSubsystem.getCoralStationLocationPose3d().getRotation().getY() > 0.5) {
            rotation = -SmartDashboard.getNumber("AutoScanSpeed", 1.0);
            RobotContainer.drivebase.drive(new Translation2d(distance, 0), rotation, false);
          } else if (VisionSubsystem.getCoralStationLocationPose3d().getRotation().getY() < -0.5) {
            rotation = SmartDashboard.getNumber("AutoScanSpeed", 1.0);
            RobotContainer.drivebase.drive(new Translation2d(distance, 0), rotation, false);
          } else {
            RobotContainer.drivebase.drive(new Translation2d(distance, 0), 0, false);
          }
        } else {
          distance = SmartDashboard.getNumber("AutoMoveSpeed", 5);
          RobotContainer.drivebase.drive(new Translation2d(distance, 0), 0, false);
        }
      }
    }
  }

  @Override
  public boolean isFinished() {
    return (VisionSubsystem.DistanceToReef() == -1)
        && (VisionSubsystem.DistanceToCoralStation() == -1)
        && (VisionSubsystem.DistanceToProcessor() == -1);
  }
}
