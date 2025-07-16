package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.LimelightHelpers.LimelightTarget_Fiducial;
import frc.robot.LimelightHelpers.RawFiducial;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import java.util.logging.Level;
import java.util.logging.Logger;

public class VisionSubsystem extends SubsystemBase {

  public static String name = frc.robot.Constants.VisionSubsystemConstants.limelightName;

  // pipeline layout:
  // 0 - april tags
  // 1 - Reef Target
  // 2 - Coral Station Target
  // 3 - Color for Algae

  // Purely from limelight's stuff

  // private final SwerveDrivePoseEstimator m_poseEstimator;

  private static int[] reefTags = {6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22};
  private static int[] coralTags = {1, 2, 12, 13};
  private static int[] processorTags = {3, 16};
  private static int[] bargeTags = {4, 5, 14, 15};
  boolean doRejectUpdate;

  public VisionSubsystem() {

    // m_poseEstimator =
    //    new SwerveDrivePoseEstimator(
    //        m_SwerveSubsystem.getKinematics(),
    //        m_SwerveSubsystem.getGyro().getRotation3d().toRotation2d(),
    //        m_SwerveSubsystem.getSwerveDrive().getModulePositions(),
    //        new Pose2d(),
    //        VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
    //        VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));
  }

  /** Updates our position on the field using seen AprilTags */
  public void UpdatePositionOnField() {

    LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);

    if (mt1.tagCount == 0) {
      doRejectUpdate = true;
    } else {
      doRejectUpdate = false;
    }
    if (mt1.tagCount == 1 && mt1.rawFiducials.length == 1) {
      if (mt1.rawFiducials[0].ambiguity > .7) {
        doRejectUpdate = true;
      }
      if (mt1.rawFiducials[0].distToCamera > 3) {
        doRejectUpdate = true;
      }
    }

    if (!doRejectUpdate) {
      RobotContainer.drivebase.addVisionReading(mt1.pose, mt1.timestampSeconds);
    }

    /*
    // use seen tags to find our position using megaTag2
    LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);

    // if this value is null then we don't see any tags or have an error somewhere
    // so if it is null or has no tags then reject updates
    if (mt2 != null) {
      if (mt2.tagCount == 0) {
        doRejectUpdate = true;
      } else {
        doRejectUpdate = false;
      }

      // add a vision reading to YAGSL if everything is in check
      if (!doRejectUpdate) {
        RobotContainer.drivebase.addVisionReading(mt2.pose, mt2.timestampSeconds);
      }*/
  }

  public void getPose3d() {
    // Get the 3d position of the robot on the field to the logger.
    Logger.getGlobal()
        .log(
            Level.WARNING,
            "QWERT Current position is - X: "
                + LimelightHelpers.getBotPoseEstimate_wpiBlue(name).pose.getX()
                + " Y: "
                + LimelightHelpers.getBotPoseEstimate_wpiBlue(name).pose.getY());
  }

  /**
   * Gets the area of the lowest ID seen AprilTag
   *
   * @return Area of lowest ID in view AprilTag
   */
  public static double getTagArea() {
    LimelightHelpers.setPipelineIndex(name, 0);
    LimelightResults results = LimelightHelpers.getLatestResults(name);
    return results.targets_Fiducials[0].ta;
  }

  /**
   * Gets the distance to the lowest ID seen AprilTag
   *
   * @return How far away the robot is from the lowest in view AprilTag
   */
  public static double getXDistance() {
    // change the pipeline to apriltags
    LimelightHelpers.setPipelineIndex(
        frc.robot.Constants.VisionSubsystemConstants.limelightName, 0);

    // get the results
    RawFiducial[] fiducials =
        LimelightHelpers.getRawFiducials(
            frc.robot.Constants.VisionSubsystemConstants.limelightName);

    return fiducials[0].distToRobot;
  }

  public static Pose2d getTagPose2d(int tag) {
    LimelightTarget_Fiducial[] fiducials =
        LimelightHelpers.getLatestResults(name).targets_Fiducials;
    for (int i = 0; i < fiducials.length; i++) {
      if (fiducials[i].fiducialID == tag) {
        return fiducials[i].getTargetPose_RobotSpace2D();
      }
    }
    return null;
  }

  /**
   * Gets the Pose2D information of the lowest seen AprilTag
   *
   * @return Lowest ID Pose2D of in view AprilTags in robot Space
   */
  public static Pose2d getTagPose2d() {
    // change the pipeline to apriltags
    LimelightHelpers.setPipelineIndex(name, 0);

    // get the results
    LimelightResults results = LimelightHelpers.getLatestResults(name);

    // if the limelights intel is good look for reef tag
    while (!results.valid) {
      results = LimelightHelpers.getLatestResults(name);
    }

    // get the first listed aprilTag and return its pose in robot space
    return results.targets_Fiducials[0].getTargetPose_RobotSpace2D();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("ReefDistance", VisionSubsystem.DistanceToReef());
    SmartDashboard.putNumber("CoralStationDistance", VisionSubsystem.DistanceToCoralStation());
    SmartDashboard.putNumber("ProcessorDistance", VisionSubsystem.DistanceToProcessor());

    UpdatePositionOnField();

    // LimelightHelpers.SetRobotOrientation(
    //    name, RobotContainer.drivebase.getHeading().getDegrees(), 0, 0, 0, 0, 0);
    /*
    LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
    if (mt2 != null) {
      if (mt2.tagCount == 0) {
        doRejectUpdate = true;
      } else {
        doRejectUpdate = false;
      }
      if (!doRejectUpdate
          && (RobotContainer.drivebase.getRobotVelocity().vxMetersPerSecond == 0
              && RobotContainer.drivebase.getRobotVelocity().vyMetersPerSecond == 0)) {
        RobotContainer.drivebase.addVisionReading(mt2.pose, mt2.timestampSeconds);
      }
    }
      */
  }

  /**
   * Gets the IDs of all in view AprilTags
   *
   * @return Tag IDs of all seen AprilTags
   */
  public static int[] getTags() {

    // set the pipeline index to AprilTags
    LimelightHelpers.setPipelineIndex(name, 0);

    // get the results from the limelight
    LimelightResults results = LimelightHelpers.getLatestResults(name);

    // if the results are not valid poll the limelight till they are
    while (!results.valid) {
      results = LimelightHelpers.getLatestResults(name);
    }

    // create an array with the length being the amount of AprilTags we can see
    int[] temp = new int[results.targets_Fiducials.length];

    // put all the in sight AprilTags into the array created above
    for (int i = 0; i < results.targets_Fiducials.length; i++) {
      temp[i] = (int) results.targets_Fiducials[i].fiducialID;
    }

    // return that array
    return temp;
  }

  /**
   * Checks to see if we can see an AprilTag
   *
   * @param int Tag ID to look for
   * @return If we can see it
   */
  public static boolean CanSeeTag(int tag) {

    // set the pipeline index to AprilTags
    LimelightHelpers.setPipelineIndex(name, 0);

    // get the results from the LimeLight
    LimelightResults results = LimelightHelpers.getLatestResults(name);

    // if the results we have are not valid then poll the LimeLight till they are
    while (!results.valid) {
      results = LimelightHelpers.getLatestResults(name);
    }

    // look through all AprilTags we can see to find the tag we are looking for
    for (LimelightTarget_Fiducial SeenTag : results.targets_Fiducials) {

      // if we find the ID in the list then we can see it and can return true
      if (SeenTag.fiducialID == tag) {
        return true;
      }
    }

    // if we have made it to the end of the list and have not found the ID
    // then we must not be able to see it and should return false
    return false;
  }

  /**
   * Finds the robots pose in field space using a seen AprilTag
   *
   * @return The position of the robot in field space
   */
  public static Pose3d getRobotPoseInFieldSpace() {

    // make sure we are not in the sim
    if (!Robot.isSimulation()) {

      // set the pipeline index to AprilTags
      LimelightHelpers.setPipelineIndex(name, 0);

      // get the results from the LimeLight
      LimelightResults results = LimelightHelpers.getLatestResults(name);

      // if the limelights intel is bad then poll it till its good
      while (!results.valid) {
        results = LimelightHelpers.getLatestResults(name);
      }

      // get the first tag we can see
      LimelightTarget_Fiducial tag = results.targets_Fiducials[0];

      // return our pose in field space
      return tag.getRobotPose_FieldSpace();
    } else {

      // if we are in the sim return null
      // as not to place us in narnia
      return null;
    }
  }

  /**
   * Finds if we can see an Algae
   *
   * @return If there is an Algae in our field of view
   */
  public static boolean CanSeeAlgae() {

    // set the pipeline index to color/reflective
    LimelightHelpers.setPipelineIndex(
        frc.robot.Constants.VisionSubsystemConstants.limelightName, 3);

    // if our target color is in view then return true
    if (LimelightHelpers.getTargetColor(name)[0] != -1) {
      return true;
    }

    // if not then false
    return false;
  }

  /**
   * Gets the X and Y of the lowest ID reef Tag in view
   *
   * @return [0] X of the Reef relative to the camera
   * @return [1] Y of the Reef relative to the camera
   * @see #getReefLocationPose3d() Return a robot relative Pose3D instead
   * @see #DistanceToReef() Return the Distance from the robot to the reef tag
   */
  public static double[] getReefLocation() {
    // change the pipeline to apriltags
    LimelightHelpers.setPipelineIndex(name, 0);

    // get the results
    LimelightResults results = LimelightHelpers.getLatestResults(name);
    Pose3d tagPoseRobot = null;

    // if the limelights intel is good look for reef tag
    while (!results.valid) {
      results = LimelightHelpers.getLatestResults(name);
    }

    // loop through all tags in the view of limelight
    for (LimelightTarget_Fiducial tag : results.targets_Fiducials) {

      // find out if any of the tags we have are those of the reef
      for (int reeftag : reefTags) {
        if (tag.fiducialID == reeftag) {

          // if we have found a reef tag break out
          tagPoseRobot = tag.getTargetPose_CameraSpace();
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

  /**
   * Gets the X and Y of the lowest ID Coral Station Tag in view
   *
   * @return [0] X of the Coral Station relative to the robot
   * @return [1] Y of the Coral Station relative to the robot
   * @see #getCoralStationLocationPose3d() Return a robot relative Pose3D instead
   * @see #DistanceToCoralStation() Return the distance from the robot to the Coral Station tag
   */
  public static double[] getCoralStationLocation() {

    // change the pipeline to apriltags
    LimelightHelpers.setPipelineIndex(name, 0);

    // get the results
    LimelightResults results = LimelightHelpers.getLatestResults(name);
    Pose3d tagPoseRobot = null;

    // if the limelights intel is good look for reef tag
    while (!results.valid) {
      results = LimelightHelpers.getLatestResults(name);
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
    if (tagPoseRobot != null) {
      return new double[] {tagPoseRobot.getX(), tagPoseRobot.getY()};
    } else {
      return new double[] {-1.0, -1.0};
    }
  }

  /**
   * Gets the X and Y of the lowest ID Processor Tag in view
   *
   * @return [0] X of the Processor relative to the robot
   * @return [1] Y of the Processor relative to the robot
   * @see #getProcessorLocationPose3d() Return a robot relative Pose3D instead
   * @see #DistanceToProcessor() Return the distance from the robot to the processor tag
   */
  public static double[] getProcessorLocation() {
    // change the pipeline to apriltags
    LimelightHelpers.setPipelineIndex(name, 0);

    // get the results
    LimelightResults results = LimelightHelpers.getLatestResults(name);
    Pose3d tagPoseRobot = null;

    // if the limelights intel is good look for reef tag
    while (!results.valid) {
      results = LimelightHelpers.getLatestResults(name);
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

  /**
   * Gets the 3D pose of the lowest ID reef Tag in view
   *
   * @return Pose of the reef relative to the robot
   * @see #getReefLocation() Return a camera relative X and Y position instead
   * @see #DistanceToReef() Return the distance from the robot to the Reef tag
   */
  public static Pose3d getReefLocationPose3d() {
    // change the pipeline to apriltags
    LimelightHelpers.setPipelineIndex(name, 0);

    // get the results
    LimelightResults results = LimelightHelpers.getLatestResults(name);
    Pose3d tagPoseRobot = null;

    // if the limelights intel is good look for reef tag
    while (!results.valid) {
      results = LimelightHelpers.getLatestResults(name);
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
      return tagPoseRobot;
    } else {
      return null;
    }
  }

  /**
   * Gets the 3D pose of the lowest ID Coral Station Tag in view
   *
   * @return Pose of the Coral Station relative to the robot
   * @see #getCoralStationLocation() Return a robot relative X and Y position instead
   * @see #DistanceToCoralStation() Return the distance from the robot to the Coral Station tag
   */
  public static Pose3d getCoralStationLocationPose3d() {
    // change the pipeline to apriltags
    LimelightHelpers.setPipelineIndex(name, 0);

    // get the results
    LimelightResults results = LimelightHelpers.getLatestResults(name);
    Pose3d tagPoseRobot = null;

    // if the limelights intel is good look for reef tag
    while (!results.valid) {
      results = LimelightHelpers.getLatestResults(name);
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
    if (tagPoseRobot != null) {
      return tagPoseRobot;
    } else {
      return null;
    }
  }

  /**
   * Gets the 3D pose of the lowest ID Processor Tag in view
   *
   * @return Pose of the Processor relative to the robot
   * @see #getProcessorLocation() Return a camera relative X and Y position instead
   * @see #DistanceToProcessor() Return the distance from the robot to the Processor tag
   */
  public static Pose3d getProcessorLocationPose3d() {
    // change the pipeline to apriltags
    LimelightHelpers.setPipelineIndex(name, 0);

    // get the results
    LimelightResults results = LimelightHelpers.getLatestResults(name);
    Pose3d tagPoseRobot = null;

    // if the limelights intel is good look for reef tag
    while (!results.valid) {
      results = LimelightHelpers.getLatestResults(name);
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
      return tagPoseRobot;
    } else {
      return null;
    }
  }

  /**
   * Gets the distance to the lowest ID reef tag in view
   *
   * @return Lowest ID reef tag distance
   * @see #getReefLocation() Return the X and Y of the reef tag from the perspective of the camera
   * @see #getReefLocationPose3d() Gives the exact position of the reef tag relative to the robot
   */
  public static double DistanceToReef() {
    // change the pipeline to apriltags
    LimelightHelpers.setPipelineIndex(
        frc.robot.Constants.VisionSubsystemConstants.limelightName, 0);

    // get the results
    RawFiducial[] fiducials =
        LimelightHelpers.getRawFiducials(
            frc.robot.Constants.VisionSubsystemConstants.limelightName);

    // make the variable to hold the shortest distance start it at the max value for doubles
    double shortest = Double.MAX_VALUE;

    // loop through all results
    for (RawFiducial fiducial : fiducials) {
      int id = fiducial.id; // Tag ID
      double distToRobot = fiducial.distToRobot; // Distance to robot

      // loop through all reef tags to find if this is a reef tag
      for (int i = 0; i < reefTags.length; i++) {

        // if it is then make it the new shortest
        if (id == reefTags[i] && distToRobot < shortest) {
          shortest = distToRobot;
        }
      }
    }

    // if the shortest has not changed then return -1 else return the shortest distance
    if (shortest != Double.MAX_VALUE) {
      return shortest;
    } else {
      return -1;
    }
  }

  /**
   * Gets the distance to the lowest ID Coral Station tag in view
   *
   * @return Lowest ID Coral Station tag distance
   * @see #getCoralStationLocation() Return the X and Y of the Coral Station tag relative to the
   *     robot
   * @see #getCoralStationLocationPose3d() Return the exact position of the Coral Station tag
   *     relative to the robot
   */
  public static double DistanceToCoralStation() {
    // change the pipeline to apriltags
    LimelightHelpers.setPipelineIndex(name, 0);

    // get the results
    RawFiducial[] fiducials = LimelightHelpers.getRawFiducials(name);

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

    // if the shortest has not changed then return -1 else return the shortest distance
    if (shortest != Double.MAX_VALUE) {
      return shortest;
    } else {
      return -1;
    }
  }

  /**
   * Gets the distance to the lowest ID Processor tag in view
   *
   * @return Lowest ID Processor tag distance
   * @see #getProcessorLocation() Return the X and Y of the Processor tag relative to the robot
   * @see #getProcessorLocationPose3d() Return the exact position of the Processor tag relative to
   *     the robot
   */
  public static double DistanceToProcessor() {
    // change the pipeline to apriltags
    LimelightHelpers.setPipelineIndex(name, 0);

    // get the results
    RawFiducial[] fiducials = LimelightHelpers.getRawFiducials(name);

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

    // if the shortest has not changed then return -1 else return the shortest distance
    if (shortest != Double.MAX_VALUE) {
      return shortest;
    } else {
      return -1;
    }
  }

  /**
   * Gets the distance to the lowest ID Barge tag in view
   *
   * @return Lowest ID Barge tag distance
   */
  public static double DistanceToBarge() {
    // change the pipeline to apriltags
    LimelightHelpers.setPipelineIndex(name, 0);

    // get the results
    RawFiducial[] fiducials = LimelightHelpers.getRawFiducials(name);

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
    // if the shortest has not changed then return -1 else return the shortest distance
    if (shortest != Double.MAX_VALUE) {
      return shortest;
    } else {
      return -1;
    }
  }
}
