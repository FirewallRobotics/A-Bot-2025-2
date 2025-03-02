package frc.robot.commands;

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

  // add vision as a requirement to run
  public AlignWithNearest(VisionSubsystem visionSubsystem) {
    addRequirements(visionSubsystem);
  }

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
