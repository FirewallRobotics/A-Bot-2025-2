package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;
import frc.robot.subsystems.VisionSubsystem;
import java.util.function.DoubleSupplier;
import java.util.logging.Level;
import java.util.logging.Logger;

public class AlignWithNearest extends Command {

  public static String name = frc.robot.Constants.VisionSubsystemConstants.limelightName;

  // private Pose2d targetPose;
  // private double distanceAway = -0.55;

  public static Pose2d[] TagPos = {
    new Pose2d(16.408, 1.048, new Rotation2d(-0.9075712)),
    new Pose2d(16.296, 7.007, new Rotation2d(0.9075712)),
    new Pose2d(11.434, 7.398, new Rotation2d(1.570796)),
    new Pose2d(0, 0, new Rotation2d(0)),
    new Pose2d(0, 0, new Rotation2d(0)),
    new Pose2d(13.69, 2.4, new Rotation2d(2.146755)),
    new Pose2d(14.261, 2.220, new Rotation2d(Math.toRadians(125))),
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

  public static Pose2d Tag6 = new Pose2d(13.69, 2.4, new Rotation2d(2.146755));
  public static Pose2d Tag7 = new Pose2d(14.538, 3.969, new Rotation2d(3.141593));
  public static Pose2d Tag8 = new Pose2d(13.840, 5.217, new Rotation2d(-2.111848));
  public static Pose2d Tag9 = new Pose2d(12.365, 5.165, new Rotation2d(-1.012291));
  public static Pose2d Tag10 = new Pose2d(11.638, 4.007, new Rotation2d(0));
  public static Pose2d Tag11 = new Pose2d(12.390, 2.790, new Rotation2d(1.012291));

  public Command targetCommand;

  /**
   * Gets the location of a reef tag in field space with an offset
   *
   * @return The location of the nearest reef tag in field space
   */
  public Pose2d getReefLocationInFieldSpaceWithOffset(double yOffset, Pose3d reeflocation) {

    double robotrot;
    double reefrot;

    // get reef location in robot space
    Pose2d reefLocationpPose2d = reeflocation.toPose2d();
    Pose2d RobotFieldSpace;

    // if we dont have the reefs location find it by spinning slowly
    if (reefLocationpPose2d == null) {
      DoubleSupplier scanspeed = () -> SmartDashboard.getNumber("AutoScanSpeed", 1.0);
      RobotContainer.drivebase.driveCommand(() -> 0, () -> 0, scanspeed).schedule();
    } else {
      // if we do have the reefs location then convert it
      // first zero the drivecommand so the math stays right
      RobotContainer.drivebase.driveCommand(() -> 0, () -> 0, () -> 0).schedule();

      // get the robots location in field space
      RobotFieldSpace = LimelightHelpers.getBotPose2d(name);

      if (reefLocationpPose2d.getRotation().getDegrees() > 180) {
        reefrot = reefLocationpPose2d.getRotation().getDegrees() - 360;
      } else {
        reefrot = reefLocationpPose2d.getRotation().getDegrees();
      }

      if (RobotFieldSpace.getRotation().getDegrees() > 180) {
        robotrot = RobotFieldSpace.getRotation().getDegrees() - 360;
      } else {
        robotrot = RobotFieldSpace.getRotation().getDegrees();
      }

      // do the math to find the location of the reef by adding together the values
      double xActual = ((-reefLocationpPose2d.getX()) + RobotFieldSpace.getX());
      double yActual = ((-(reefLocationpPose2d.getY() + yOffset)) + RobotFieldSpace.getY());
      double rotActual = ((-reefrot) + robotrot);

      if (rotActual < 0) {
        rotActual += 360;
      }

      // return the values
      SmartDashboard.putNumberArray("LocationCalcu", new Double[] {xActual, yActual, rotActual});
      return new Pose2d(new Translation2d(xActual, yActual), new Rotation2d(rotActual));
    }
    return null;
  }

  // add vision as a requirement to run
  public AlignWithNearest() {}

  int TagAligningToo;
  ConditionalCommand conditionalDriveCommand;

  @Override
  public void initialize() {
    int[] tags = VisionSubsystem.getTags();
    if (tags.length > 0) {
      TagAligningToo = tags[0];
      // Provides direct connection that bypasses pathplanner (more accurate)
      // If we use this, put it in execute and have it change constantly to update as data streams
      // in.
      Pose2d TagLocation = VisionSubsystem.getTagPose2d(TagAligningToo);
      if (TagLocation != null) {
        Logger.getGlobal()
            .log(
                Level.INFO,
                (TagLocation.getX())
                    + " "
                    + (TagLocation.getY())
                    + " "
                    + (TagLocation.getRotation().getRotations()));
        // RobotContainer.drivebase.drive(new Translation2d(0.5, 0), 0, false);
        Command driveCommand =
            RobotContainer.drivebase.driveCommand(
                () -> -(TagLocation.getX()),
                () -> -(TagLocation.getY()),
                () -> -(TagLocation.getRotation().getRotations()));
        conditionalDriveCommand =
            driveCommand.unless(
                () ->
                    (RobotContainer.driverXbox.back().getAsBoolean()
                        || RobotContainer.coralController.back().getAsBoolean()));
        conditionalDriveCommand.schedule();
      }
    } else {
      Command driveCommand = RobotContainer.drivebase.driveCommand(() -> 0, () -> 0, () -> 0);
      conditionalDriveCommand =
          driveCommand.unless(
              () ->
                  (RobotContainer.driverXbox.back().getAsBoolean()
                          || RobotContainer.coralController.back().getAsBoolean())
                      || !RobotContainer.coralController.rightBumper().getAsBoolean());
      conditionalDriveCommand.schedule();
    }
  }

  @Override
  public void execute() {
    int[] tags = VisionSubsystem.getTags();
    if (tags.length > 0 && conditionalDriveCommand.isFinished()) {
      TagAligningToo = tags[0];
      // Provides direct connection that bypasses pathplanner (more accurate)
      // If we use this, put it in execute and have it change constantly to update as data streams
      // in.
      Pose2d TagLocation = VisionSubsystem.getTagPose2d(TagAligningToo);
      if (TagLocation != null) {
        Logger.getGlobal()
            .log(
                Level.INFO,
                (TagLocation.getX())
                    + " "
                    + (TagLocation.getY())
                    + " "
                    + (TagLocation.getRotation().getRotations()));
        // RobotContainer.drivebase.drive(new Translation2d(0.5, 0), 0, false);
        Command driveCommand =
            RobotContainer.drivebase.driveCommand(
                () -> -(TagLocation.getX()),
                () -> -(TagLocation.getY()),
                () -> -(TagLocation.getRotation().getRotations()));
        conditionalDriveCommand =
            driveCommand.unless(
                () ->
                    (RobotContainer.driverXbox.back().getAsBoolean()
                            || RobotContainer.coralController.back().getAsBoolean())
                        || !RobotContainer.coralController.rightBumper().getAsBoolean());
        conditionalDriveCommand.schedule();
      }
    }
  }

  @Override
  public boolean isFinished() {
    return !VisionSubsystem.CanSeeTag(TagAligningToo)
        || (RobotContainer.driverXbox.back().getAsBoolean()
            || RobotContainer.coralController.back().getAsBoolean())
        || !RobotContainer.coralController.rightBumper().getAsBoolean();
  }
}
