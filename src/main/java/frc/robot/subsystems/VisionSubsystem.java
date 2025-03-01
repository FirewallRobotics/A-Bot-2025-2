package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.LimelightHelpers.LimelightTarget_Detector;
import frc.robot.LimelightHelpers.LimelightTarget_Fiducial;
import frc.robot.LimelightHelpers.RawFiducial;
import frc.robot.Robot;

public class VisionSubsystem extends SubsystemBase {

  // pipeline layout:
  // 0 - april tags
  // 1 - Coral / white mask
  // 2 - Algae / Circlular Green mask

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
    final Field2d m_field = new Field2d();
    // Do this in either robot or subsystem init
    SmartDashboard.putData("Field", m_field);
    // Do this in either robot periodic or subsystem periodic
    LimelightResults results = LimelightHelpers.getLatestResults("");
    LimelightTarget_Fiducial tag = results.targets_Fiducials[0];
    Pose3d temp = tag.getRobotPose_FieldSpace();
    Translation2d tempTrans = new Translation2d(temp.getX(), temp.getY());
    Rotation2d temprot = new Rotation2d(temp.getRotation().getX(), temp.getRotation().getY());
    m_field.setRobotPose(new Pose2d(tempTrans, temprot));
    SmartDashboard.putData("Field", m_field);
  }

  public static Pose3d getRobotPoseInFieldSpace() {
    if (!Robot.isSimulation()) {
      LimelightHelpers.setPipelineIndex("", 0);
      LimelightResults results = LimelightHelpers.getLatestResults("");
      // if the limelights intel is good look for reef tag
      while (!results.valid) {
        results = LimelightHelpers.getLatestResults("");
      }
      LimelightTarget_Fiducial tag = results.targets_Fiducials[0];
      return tag.getRobotPose_FieldSpace();
    } else {
      return null;
    }
  }

  public static double[] getCoralLocation() {
    LimelightHelpers.setPipelineIndex("", 1);
    return LimelightHelpers.getTargetPose_RobotSpace("");
  }

  public static double[] getAlgaeLocation() {
    LimelightHelpers.setPipelineIndex("", 2);
    return LimelightHelpers.getTargetPose_RobotSpace("");
  }

  public static double[] getCoralLocationCamera() {
    // get the pipeline used before and save it for after we have finished our work
    int pipelineTempdex = (int) LimelightHelpers.getCurrentPipelineIndex("");

    // change the pipeline to the game object AI
    LimelightHelpers.setPipelineIndex("", 1);

    // get the results from the AI
    LimelightResults results = LimelightHelpers.getLatestResults("");

    // if it has results then loop through them
    if (results.targets_Detector.length > 0) {
      for (int i = 0; i < results.targets_Detector.length; i++) {
        LimelightTarget_Detector detection = results.targets_Detector[i];

        // if this result is a coral then return its location
        if (detection.className.equals("coral")) {

          // set pipeline to the what it was before
          LimelightHelpers.setPipelineIndex("", pipelineTempdex);
          return new double[] {detection.tx, detection.ty};
        }
      }
    }

    // set pipeline to the what it was before
    LimelightHelpers.setPipelineIndex("", pipelineTempdex);
    return null;
  }

  public static double[] getAlgaeLocationCamera() {
    // get the pipeline used before and save it for after we have finished our work
    int pipelineTempdex = (int) LimelightHelpers.getCurrentPipelineIndex("");

    // change the pipeline to the game object AI
    LimelightHelpers.setPipelineIndex("", 1);

    // get the results from the AI
    LimelightResults results = LimelightHelpers.getLatestResults("");

    // if it has results then loop through them
    if (results.targets_Detector.length > 0) {
      for (int i = 0; i < results.targets_Detector.length; i++) {
        LimelightTarget_Detector detection = results.targets_Detector[i];

        // set pipeline to the what it was before
        if (detection.className.equals("algae")) {

          // set pipeline to the what it was before
          LimelightHelpers.setPipelineIndex("", pipelineTempdex);
          return new double[] {detection.tx, detection.ty};
        }
      }
    }

    // set pipeline to the what it was before
    LimelightHelpers.setPipelineIndex("", pipelineTempdex);
    return null;
  }

  public static double[] getReefLocation() {
    // get the pipeline used before and save it for after we have finished our work
    int pipelineTempdex = (int) LimelightHelpers.getCurrentPipelineIndex("");

    // change the pipeline to apriltags
    LimelightHelpers.setPipelineIndex("", 0);

    // get the results
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
    // set pipeline to the what it was before
    LimelightHelpers.setPipelineIndex("", pipelineTempdex);
    if (tagPoseRobot != null) {
      return new double[] {tagPoseRobot.getX(), tagPoseRobot.getY()};
    } else {
      return new double[] {-1.0, -1.0};
    }
  }

  public static double[] getCoralStationLocation() {
    // get the pipeline used before and save it for after we have finished our work
    int pipelineTempdex = (int) LimelightHelpers.getCurrentPipelineIndex("");

    // change the pipeline to apriltags
    LimelightHelpers.setPipelineIndex("", 0);

    // get the results
    LimelightResults results = LimelightHelpers.getLatestResults("");
    Pose3d tagPoseRobot = null;

    // if the limelights intel is good look for reef tag
    while (!results.valid) {
      results = LimelightHelpers.getLatestResults("");
    }

    // loop through all tags in the view of limelight
    for (LimelightTarget_Fiducial tag : results.targets_Fiducials) {

      // find out if any of the tags we have are those of the reef
      for (int coraltag : coralTags) {
        if (tag.fiducialID == coraltag) {

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
    // set pipeline to the what it was before
    LimelightHelpers.setPipelineIndex("", pipelineTempdex);
    if (tagPoseRobot != null) {
      return new double[] {tagPoseRobot.getX(), tagPoseRobot.getY()};
    } else {
      return new double[] {-1.0, -1.0};
    }
  }

  public static double[] getProcessorLocation() {
    // get the pipeline used before and save it for after we have finished our work
    int pipelineTempdex = (int) LimelightHelpers.getCurrentPipelineIndex("");

    // change the pipeline to apriltags
    LimelightHelpers.setPipelineIndex("", 0);

    // get the results
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
    // set pipeline to the what it was before
    LimelightHelpers.setPipelineIndex("", pipelineTempdex);
    if (tagPoseRobot != null) {
      return new double[] {tagPoseRobot.getX(), tagPoseRobot.getY()};
    } else {
      return new double[] {-1.0, -1.0};
    }
  }

  public static double DistanceToReef() {
    // get the pipeline used before and save it for after we have finished our work
    int pipelineTempdex = (int) LimelightHelpers.getCurrentPipelineIndex("");

    // change the pipeline to apriltags
    LimelightHelpers.setPipelineIndex("", 0);

    // get the results
    RawFiducial[] fiducials = LimelightHelpers.getRawFiducials("");

    // make the variable to hold the shortest distance start it at the max value for doubles
    double shortest = Double.MAX_VALUE;

    // loop through all results
    for (RawFiducial fiducial : fiducials) {
      int id = fiducial.id; // Tag ID
      double distToRobot = fiducial.distToRobot; // Distance to robot

      // loop through all reef tags to find if this is a reef tag
      for (int i = 0; i < reefTags.length; i++) {

        // if it is then make it the new shortest
        if (id == reefTags[i]) {
          shortest = distToRobot;
        }
      }
    }

    // set pipeline to the what it was before
    LimelightHelpers.setPipelineIndex("", pipelineTempdex);

    // if the shortest has not changed then return -1 else return the shortest distance
    if (shortest != Double.MAX_VALUE) {
      return shortest;
    } else {
      return -1;
    }
  }

  public static double DistanceToCoralStation() {
    // get the pipeline used before and save it for after we have finished our work
    int pipelineTempdex = (int) LimelightHelpers.getCurrentPipelineIndex("");

    // change the pipeline to apriltags
    LimelightHelpers.setPipelineIndex("", 0);

    // get the results
    RawFiducial[] fiducials = LimelightHelpers.getRawFiducials("");

    // make the variable to hold the shortest distance start it at the max value for doubles
    double shortest = Double.MAX_VALUE;

    // loop through all results
    for (RawFiducial fiducial : fiducials) {
      int id = fiducial.id; // Tag ID
      double distToRobot = fiducial.distToRobot; // Distance to robot

      // loop through all coral Station tags to find if this is a coral Station tag
      for (int i = 0; i < coralTags.length; i++) {

        // if it is then make it the new shortest
        if (id == coralTags[i]) {
          shortest = distToRobot;
        }
      }
    }
    // set pipeline to the what it was before
    LimelightHelpers.setPipelineIndex("", pipelineTempdex);

    // if the shortest has not changed then return -1 else return the shortest distance
    if (shortest != Double.MAX_VALUE) {
      return shortest;
    } else {
      return -1;
    }
  }

  public static double DistanceToProcessor() {
    // get the pipeline used before and save it for after we have finished our work
    int pipelineTempdex = (int) LimelightHelpers.getCurrentPipelineIndex("");

    // change the pipeline to apriltags
    LimelightHelpers.setPipelineIndex("", 0);

    // get the results
    RawFiducial[] fiducials = LimelightHelpers.getRawFiducials("");

    // make the variable to hold the shortest distance start it at the max value for doubles
    double shortest = Double.MAX_VALUE;

    // loop through all results
    for (RawFiducial fiducial : fiducials) {
      int id = fiducial.id; // Tag ID
      double distToRobot = fiducial.distToRobot; // Distance to robot

      // loop through all processor tags to find if this is a processor tag
      for (int i = 0; i < processorTags.length; i++) {

        // if it is then make it the new shortest
        if (id == processorTags[i]) {
          shortest = distToRobot;
        }
      }
    }
    // set pipeline to the what it was before
    LimelightHelpers.setPipelineIndex("", pipelineTempdex);

    // if the shortest has not changed then return -1 else return the shortest distance
    if (shortest != Double.MAX_VALUE) {
      return shortest;
    } else {
      return -1;
    }
  }

  public static double DistanceToBarge() {
    // get the pipeline used before and save it for after we have finished our work
    int pipelineTempdex = (int) LimelightHelpers.getCurrentPipelineIndex("");

    // change the pipeline to apriltags
    LimelightHelpers.setPipelineIndex("", 0);

    // get the results
    RawFiducial[] fiducials = LimelightHelpers.getRawFiducials("");

    // make the variable to hold the shortest distance start it at the max value for doubles
    double shortest = Double.MAX_VALUE;

    // loop through all results
    for (RawFiducial fiducial : fiducials) {
      int id = fiducial.id; // Tag ID
      double distToRobot = fiducial.distToRobot; // Distance to robot

      // loop through all barge tags to find if this is a barge tag
      for (int i = 0; i < bargeTags.length; i++) {

        // if it is then make it the new shortest
        if (id == bargeTags[i]) {
          shortest = distToRobot;
        }
      }
    }
    // set pipeline to the what it was before
    LimelightHelpers.setPipelineIndex("", pipelineTempdex);
    // if the shortest has not changed then return -1 else return the shortest distance
    if (shortest != Double.MAX_VALUE) {
      return shortest;
    } else {
      return -1;
    }
  }
}
