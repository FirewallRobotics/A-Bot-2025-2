package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.LimelightHelpers.LimelightTarget_Detector;
import frc.robot.LimelightHelpers.LimelightTarget_Fiducial;
import frc.robot.LimelightHelpers.RawDetection;
import frc.robot.LimelightHelpers.RawFiducial;

public class VisionSubsystem extends SubsystemBase {

  // pipeline layout:
  // 0 - april tags
  // 1 - Neural network coral/algae

  private static int[] reefTags = {6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22};
  private static int[] coralTags = {1, 2, 12, 13};
  private static int[] processorTags = {3, 16};
  private static int[] bargeTags = {4, 5, 14, 15};

  @Override
  public void periodic() {
    // switch pipeline to april tags
    LimelightHelpers.setPipelineIndex("", 0);

    LimelightHelpers.SetFiducialIDFiltersOverride(
        "",
        new int[] {
          0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22
        });
    LimelightHelpers.SetFiducialDownscalingOverride("", 1.5f);

    // send targets to drivers
    SmartDashboard.putNumber("AprilTags Found:", LimelightHelpers.getTargetCount(getName()));

    // First, tell Limelight your robot's current orientation
    // double robotYaw = m_gyro.getYaw(); todo: when movement code works add gyro
    // LimelightHelpers.SetRobotOrientation("", robotYaw, 0.0, 0.0, 0.0, 0.0, 0.0);

  }

  public static Pose3d getRobotPoseInFieldSpace() {
    LimelightHelpers.setPipelineIndex("", 0);
    LimelightResults results = LimelightHelpers.getLatestResults("");
    // if the limelights intel is good look for reef tag
    while (!results.valid) {
      results = LimelightHelpers.getLatestResults("");
    }
    LimelightTarget_Fiducial tag = results.targets_Fiducials[0];
    return tag.getRobotPose_FieldSpace();
  }

  public static double[] getCoralLocationCamera() {
    LimelightHelpers.setPipelineIndex("", 1);
    LimelightResults results = LimelightHelpers.getLatestResults("");
    if (results.targets_Detector.length > 0) {
      for(int i = 0; i < results.targets_Detector.length; i++){
        LimelightTarget_Detector detection = results.targets_Detector[i];
        if(detection.className.equals("coral")){
          return new double[] {detection.tx, detection.ty};
        }
      }
    }
    return null;
  }

  public static double[] getAlgaeLocationCamera() {
    LimelightHelpers.setPipelineIndex("", 1);
    LimelightResults results = LimelightHelpers.getLatestResults("");
    if (results.targets_Detector.length > 0) {
      for(int i = 0; i < results.targets_Detector.length; i++){
        LimelightTarget_Detector detection = results.targets_Detector[i];
        if(detection.className.equals("algae")){
          return new double[] {detection.tx, detection.ty};
        }
      }
    }
    return null;
  }

  public static double[] getReefLocation() {
    LimelightHelpers.setPipelineIndex("", 0);
    LimelightResults results = LimelightHelpers.getLatestResults("");
    Pose3d tagPoseRobot = null;
    // if the limelights intel is good look for reef tag
    while (!results.valid) {
      results = LimelightHelpers.getLatestResults("");
    }
    // loop through all tags in the view of limelight
    for (LimelightTarget_Fiducial tag : results.targets_Fiducials) {
      // find out if any of the tags we have are those of the reef
      for (int reeftag : reefTags) {
        if (tag.fiducialID == reeftag) {
          // if we have found a reef tag break out
          tagPoseRobot = tag.getTargetPose_RobotSpace();
          break;
        }
      }
      if (tagPoseRobot != null) {
        // continue to break out if we have a reef tag
        break;
      }
    }
    // if the view of the limelight has no reef tags in return -1, -1 so that auto can scan
    if (tagPoseRobot != null) {
      return new double[] {tagPoseRobot.getX(), tagPoseRobot.getY()};
    } else {
      return new double[] {-1.0, -1.0};
    }
  }

  public static double[] getProcessorLocation(){
    LimelightHelpers.setPipelineIndex("", 0);
    LimelightResults results = LimelightHelpers.getLatestResults("");
    Pose3d tagPoseRobot = null;
    // if the limelights intel is good look for reef tag
    while (!results.valid) {
      results = LimelightHelpers.getLatestResults("");
    }
    // loop through all tags in the view of limelight
    for (LimelightTarget_Fiducial tag : results.targets_Fiducials) {
      // find out if any of the tags we have are those of the reef
      for (int processorTag : processorTags) {
        if (tag.fiducialID == processorTag) {
          // if we have found a reef tag break out
          tagPoseRobot = tag.getTargetPose_RobotSpace();
          break;
        }
      }
      if (tagPoseRobot != null) {
        // continue to break out if we have a reef tag
        break;
      }
    }
    // if the view of the limelight has no reef tags in return -1, -1 so that auto can scan
    if (tagPoseRobot != null) {
      return new double[] {tagPoseRobot.getX(), tagPoseRobot.getY()};
    } else {
      return new double[] {-1.0, -1.0};
    }
  }

  public double DistanceToReef() {
    LimelightHelpers.setPipelineIndex("", 0);
    RawFiducial[] fiducials = LimelightHelpers.getRawFiducials("");
    double shortest = Double.MAX_VALUE;
    for (RawFiducial fiducial : fiducials) {
      int id = fiducial.id; // Tag ID
      double distToRobot = fiducial.distToRobot; // Distance to robot
      for (int i = 0; i < reefTags.length; i++) {
        if (id == reefTags[i]) {
          shortest = distToRobot;
        }
      }
    }
    if (shortest != Double.MAX_VALUE) {
      return shortest;
    } else {
      return -1;
    }
  }

  public double DistanceToCoralStation() {
    LimelightHelpers.setPipelineIndex("", 0);
    RawFiducial[] fiducials = LimelightHelpers.getRawFiducials("");
    double shortest = Double.MAX_VALUE;
    for (RawFiducial fiducial : fiducials) {
      int id = fiducial.id; // Tag ID
      double distToRobot = fiducial.distToRobot; // Distance to robot
      for (int i = 0; i < coralTags.length; i++) {
        if (id == coralTags[i]) {
          shortest = distToRobot;
        }
      }
    }
    if (shortest != Double.MAX_VALUE) {
      return shortest;
    } else {
      return -1;
    }
  }

  public double DistanceToProcessor() {
    LimelightHelpers.setPipelineIndex("", 0);
    RawFiducial[] fiducials = LimelightHelpers.getRawFiducials("");
    double shortest = Double.MAX_VALUE;
    for (RawFiducial fiducial : fiducials) {
      int id = fiducial.id; // Tag ID
      double distToRobot = fiducial.distToRobot; // Distance to robot
      for (int i = 0; i < processorTags.length; i++) {
        if (id == processorTags[i]) {
          shortest = distToRobot;
        }
      }
    }
    if (shortest != Double.MAX_VALUE) {
      return shortest;
    } else {
      return -1;
    }
  }

  public double DistanceToBarge() {
    LimelightHelpers.setPipelineIndex("", 0);
    RawFiducial[] fiducials = LimelightHelpers.getRawFiducials("");
    double shortest = Double.MAX_VALUE;
    for (RawFiducial fiducial : fiducials) {
      int id = fiducial.id; // Tag ID
      double distToRobot = fiducial.distToRobot; // Distance to robot
      for (int i = 0; i < bargeTags.length; i++) {
        if (id == bargeTags[i]) {
          shortest = distToRobot;
        }
      }
    }
    if (shortest != Double.MAX_VALUE) {
      return shortest;
    } else {
      return -1;
    }
  }
}
