package frc.robot.subsystems;

import frc.robot.LimelightHelpers;

public class LimelightSubsystem {
  public static String name = frc.robot.Constants.VisionSubsystemConstants.limelightName;

  private static int[] reefTags = {6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22};
  private static int[] coralTags = {1, 2, 12, 13};
  private static int[] processorTags = {3, 16};
  private static int[] bargeTags = {4, 5, 14, 15};
  boolean doRejectUpdate;

  public void estimatePosition() {

    LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");

    if (mt1.tagCount == 1 && mt1.rawFiducials.length == 1) {
      if (mt1.rawFiducials[0].ambiguity > .7) {
        doRejectUpdate = true;
      }
      if (mt1.rawFiducials[0].distToCamera > 3) {
        doRejectUpdate = true;
      }
    }
    if (mt1.tagCount == 0) {
      doRejectUpdate = true;
    }

    /*if(!doRejectUpdate)
    {
      m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.5,.5,9999999));
      m_poseEstimator.addVisionMeasurement(
          mt1.pose,
          mt1.timestampSeconds);
    }*/
  }
}
