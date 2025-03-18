package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.VisionSubsystem;
import java.util.logging.Level;
import java.util.logging.Logger;

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

    // if we can see the coral station go to it
    if (VisionSubsystem.DistanceToCoralStation() != -1) {

      // get the pose of the coral station in robot orientation
      Pose3d coralstation = VisionSubsystem.getCoralStationLocationPose3d();

      // drive to that pose
      RobotContainer.drivebase.driveCommand(
          () -> coralstation.getX(),
          () -> coralstation.getY(),
          () -> coralstation.getRotation().getX());

      // log where we went/are going
      Logger.getGlobal().log(Level.INFO, "Driver Assist Going To Coral Station");

      // if we can see the processor go to it
    } else if (VisionSubsystem.DistanceToProcessor() != -1) {

      // get the pose of the processor in robot orientation
      Pose3d processor = VisionSubsystem.getProcessorLocationPose3d();

      // drive to that pose
      RobotContainer.drivebase.driveCommand(
          () -> processor.getX(), () -> processor.getY(), () -> processor.getRotation().getX());

      // log where we went/are going
      Logger.getGlobal().log(Level.INFO, "Driver Assist Going To Processor");

      // if we can see the reef go to it
    } else if (VisionSubsystem.DistanceToReef() != -1) {

      // get the reefs pose in robot orientation
      Pose3d reef = VisionSubsystem.getReefLocationPose3d();

      // go to that pose
      RobotContainer.drivebase.driveCommand(
          () -> reef.getX(), () -> reef.getY(), () -> reef.getRotation().getX());

      // log where we went/are going
      Logger.getGlobal().log(Level.INFO, "Driver Assist Going To Reef");
    } else {

      // log that we cannot see anything to goto
      Logger.getGlobal().log(Level.WARNING, "Driver Assist Cannot Find A Valid Target!");
    }
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
