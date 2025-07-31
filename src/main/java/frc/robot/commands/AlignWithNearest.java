package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotContainer;
import frc.robot.subsystems.VisionSubsystem;
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
    new Pose2d(11.3, 2.8, new Rotation2d(1.012291)),
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

  // Not used commented to save memory and provide visual rep of tag locations
  // public static Pose2d Tag13 = new Pose2d(1.131, 6.950, new Rotation2d(2.181662));
  // public static Pose2d Tag12 = new Pose2d(1.161, 1.048, new Rotation2d(-2.216568));
  // public static Pose2d Tag2 = new Pose2d(16.296, 7.007, new Rotation2d(0.9075712));
  // public static Pose2d Tag1 = new Pose2d(16.408, 1.048, new Rotation2d(-0.9075712));

  // public static Pose2d Tag3 = new Pose2d(11.434, 7.398, new Rotation2d(1.570796));
  // public static Pose2d Tag16 = new Pose2d(6.364, 0.550, new Rotation2d(-1.570796));

  // public static Pose2d Tag17 = new Pose2d(3.390, 2.790, new Rotation2d(1.012291));
  // public static Pose2d Tag18 = new Pose2d(2.638, 4.007, new Rotation2d(0));
  // public static Pose2d Tag19 = new Pose2d(3.365, 5.165, new Rotation2d(-1.012291));
  // public static Pose2d Tag20 = new Pose2d(4.840, 5.217, new Rotation2d(-2.111848));
  // public static Pose2d Tag21 = new Pose2d(5.538, 3.969, new Rotation2d(3.141593));
  // public static Pose2d Tag22 = new Pose2d(4.787, 2.811, new Rotation2d(2.094395));

  // public static Pose2d Tag6 = new Pose2d(13.69, 2.4, new Rotation2d(2.146755));
  // public static Pose2d Tag7 = new Pose2d(14.538, 3.969, new Rotation2d(3.141593));
  // public static Pose2d Tag8 = new Pose2d(13.840, 5.217, new Rotation2d(-2.111848));
  // public static Pose2d Tag9 = new Pose2d(12.365, 5.165, new Rotation2d(-1.012291));
  // public static Pose2d Tag10 = new Pose2d(11.638, 4.007, new Rotation2d(0));
  // public static Pose2d Tag11 = new Pose2d(12.390, 2.790, new Rotation2d(1.012291));

  public Command targetCommand;

  /** Rotation PID Controller */
  PIDController rController;

  /** X (left/right) PID Controller */
  PIDController xController;

  /** Y (FWD/Back) PID Controller */
  PIDController yController;

  /** Tag ID we are aligning too */
  int TagAligningToo;

  /** Command that is actually scheduled. Adds a condition that stops us upon being done */
  ParallelRaceGroup conditionalDriveCommand;

  /**
   * Location of the Tag updated every frame once and only once We found that the rotation is very
   * inconsistant while the X and Y are more stable
   */
  Pose2d TagLocation;

  /**
   * Amount of frames we cannot see the tag. We give some grace to losing the tag for a few frames
   */
  int GraceFrames;

  VisionSubsystem visionSubsystem;

  /** Button that has triggered this command */
  Trigger trigger;

  /** Controller that the driver holds */
  CommandXboxController driveXboxController;

  /** Frames since last rot update. We avg 5 frames of rot data to make it more consistant */
  int rotCounter;

  /**
   * Up to 5 frames of rot data added together. We avg 5 frames of rot data to make it more
   * consistant
   */
  double rotValue;

  /**
   * The last avged value of the rot. At the start we set this to rot We avg 5 frames of rot data to
   * make it more consistant
   */
  double rotValueAvg;

  /**
   * Align with the nearest tag (offset to allow us to align with the left reef stick)
   *
   * @param offset DEPRECADED UNUSED
   * @param trigger Button that triggered this command (If the trigger is released we stop the
   *     command)
   * @param driveXboxController Controller that the driver holds (used for rumble)
   */
  public AlignWithNearest(
      double offset, Trigger trigger, CommandXboxController driveXboxController) {

    // P = speed
    // I = smoothing
    // D = time
    xController = new PIDController(4, 0, 0);
    yController = new PIDController(14, 0, 0);
    rController = new PIDController(0.27, 0, 0);
    TagLocation = new Pose2d(0, 0, new Rotation2d(0));

    // Init PID values are sent to networktables so we can change it on the fly
    SmartDashboard.putData("xController", xController);
    SmartDashboard.putData("yController", yController);
    SmartDashboard.putData("rController", rController);

    visionSubsystem = new VisionSubsystem();

    this.trigger = trigger;
    this.driveXboxController = driveXboxController;
  }

  @Override
  public void initialize() {

    // Get the data for alignment before starting the command
    xController = (PIDController) SmartDashboard.getData("xController");
    yController = (PIDController) SmartDashboard.getData("yController");
    rController = (PIDController) SmartDashboard.getData("rController");

    // init the setpoint and error
    xController.setSetpoint(0);
    xController.setTolerance(0.1);
    yController.setSetpoint(0);
    yController.setTolerance(0.1);
    rController.setSetpoint(0);
    rController.setTolerance(0.01);

    // Get the first tags values and make sure we can see a tag
    int[] tags = visionSubsystem.getTagsUnchanging();

    if (tags.length > 0) {
      // The first tag we see is the tag we are aligning too
      TagAligningToo = tags[0];

      TagLocation = visionSubsystem.getTagPose2dUnchanging(TagAligningToo);

      if (TagLocation != null) {
        // If we can see a tag then set the init rot value
        rotValueAvg = TagLocation.getRotation().getDegrees();
      }
    }
  }

  @Override
  public void execute() {

    // Send the current grace frames to networktables for monitoring
    SmartDashboard.putNumber("GraceFrames", GraceFrames);

    // Get tag ids
    int[] tags = visionSubsystem.getTagsUnchanging();

    // if we can see tags
    if (tags.length > 0 && visionSubsystem.CanSeeTagUnchanging(TagAligningToo)) {
      // Provides direct connection that bypasses pathplanner (more accurate)
      // If we use this, put it in execute and have it change constantly to update as data streams
      // in.
      TagLocation = visionSubsystem.getTagPose2dUnchanging(TagAligningToo);

      GraceFrames = 0;

      // if we have less the 5 frames of rotation data add another
      if (rotCounter != 5) {
        rotValue += TagLocation.getRotation().getDegrees();
        rotCounter++;

        // if we have 5 frames then avg and push the value for use
      } else {
        rotValueAvg = rotValue / 5;
        rotValue = 0;
        rotCounter = 0;
      }

      // create the local driveCommand
      Command driveCommand = null;

      // if things go bad then bring back this logging
      // SmartDashboard.putNumberArray(
      //     "Calculated pos",
      //     new double[] {
      //       yController.calculate(
      //           TagLocation.getY() + SmartDashboard.getNumber("Y-Stop-Dist", 0.025)),
      //       xController.calculate(TagLocation.getX()),
      //       rController.calculate(
      //           MathUtil.inputModulus(TagLocation.getRotation().getDegrees(), -180, 180))
      //     });

      // drive towards the x, y, and rot value we get from the limelight.
      driveCommand =
          RobotContainer.drivebase.driveCommand(
              new ChassisSpeeds(
                  (yController.calculate(
                      TagLocation.getY() + SmartDashboard.getNumber("Y-Stop-Dist", 0.025))),
                  (xController.calculate(
                      TagLocation.getX() + SmartDashboard.getNumber("X-Stop-Dist", 0.025))),
                  (rController.calculate(
                      MathUtil.inputModulus(TagLocation.getRotation().getDegrees(), -180, 180)))));

      // if we have a trigger (not in auto)
      if (trigger != null) {

        // bring in the conditions for stopping with trigger being released
        conditionalDriveCommand =
            driveCommand.until(
                () ->
                    (TagLocation.getY()
                                    < ((-SmartDashboard.getNumber("Y-Stop-Dist", 0.025))
                                        + SmartDashboard.getNumber("yError", 0.1))
                                && TagLocation.getY()
                                    > ((-SmartDashboard.getNumber("Y-Stop-Dist", 0.025))
                                        - SmartDashboard.getNumber("yError", 0.1)))
                            && (TagLocation.getX()
                                    < ((-SmartDashboard.getNumber("X-Stop-Dist", 0.025))
                                        + SmartDashboard.getNumber("xError", 0.1))
                                && TagLocation.getX()
                                    > ((-SmartDashboard.getNumber("X-Stop-Dist", 0.025))
                                        - SmartDashboard.getNumber("xError", 0.1)))
                            && ((TagLocation.getRotation().getDegrees()
                                    < SmartDashboard.getNumber("rError", 1))
                                && (TagLocation.getRotation().getDegrees()
                                    > -SmartDashboard.getNumber("rError", 1)))
                        || !visionSubsystem.CanSeeTagUnchanging(TagAligningToo)
                        || (RobotContainer.driverXbox.back().getAsBoolean()
                            || RobotContainer.coralController.back().getAsBoolean())
                        || !trigger.getAsBoolean());

        // since we have a trigger then we can make it only schedule the command
        // if the trigger is depressed
        if (trigger.getAsBoolean()) {

          conditionalDriveCommand.schedule();

          // do the rumble if possible
          if (driveXboxController != null) {
            driveXboxController.setRumble(RumbleType.kBothRumble, 1);
          }

        } else {
          Logger.getGlobal().log(Level.WARNING, "Button release cancel");

          // cancel the rumble if possible
          if (conditionalDriveCommand != null && driveXboxController != null) {
            conditionalDriveCommand.cancel();
            driveXboxController.setRumble(RumbleType.kBothRumble, 0);
          }
        }
      } else {
        // if we don't have a trigger then we should not add in the trigger release condition
        conditionalDriveCommand =
            driveCommand.until(
                () ->
                    (TagLocation.getY()
                                    < ((-SmartDashboard.getNumber("Y-Stop-Dist", 0.025))
                                        + SmartDashboard.getNumber("yError", 0.1))
                                && TagLocation.getY()
                                    > ((-SmartDashboard.getNumber("Y-Stop-Dist", 0.025))
                                        - SmartDashboard.getNumber("yError", 0.1)))
                            && (TagLocation.getX()
                                    < ((-SmartDashboard.getNumber("X-Stop-Dist", 0.025))
                                        + SmartDashboard.getNumber("xError", 0.1))
                                && TagLocation.getX()
                                    > ((-SmartDashboard.getNumber("X-Stop-Dist", 0.025))
                                        - SmartDashboard.getNumber("xError", 0.1)))
                            && ((TagLocation.getRotation().getDegrees()
                                    < SmartDashboard.getNumber("rError", 1))
                                && (TagLocation.getRotation().getDegrees()
                                    > -SmartDashboard.getNumber("rError", 1)))
                        || !visionSubsystem.CanSeeTagUnchanging(TagAligningToo)
                        || (RobotContainer.driverXbox.back().getAsBoolean()
                            || RobotContainer.coralController.back().getAsBoolean()));

        // after setting the condition schedule it
        conditionalDriveCommand.schedule();

        // set the rumble if we have it
        if (driveXboxController != null) {
          driveXboxController.setRumble(RumbleType.kBothRumble, 1);
        }
      }

      // if we cannot see a tag check if we are above the max frames where we give grace
    } else if (GraceFrames > SmartDashboard.getNumber("MaxGraceFrames", 10)) {

      // say we are above the limit cancel the command if scheduled/exists
      Logger.getGlobal().log(Level.WARNING, "No tags cancel");
      if (conditionalDriveCommand != null) {
        conditionalDriveCommand.cancel();
      }

      // along with that stop the rumble if possible
      if (driveXboxController != null) {
        driveXboxController.setRumble(RumbleType.kBothRumble, 0);
      }

      // if not above the limit then increment graceframes and do nothing
    } else {
      GraceFrames += 1;
    }
  }

  /**
   *
   *
   * <h2>Finished if any of the following is true: </h2>
   *
   * <p>Too close on X, Y, Rot
   *
   * <p>Lost the tag and been too long
   *
   * <p>Back buttons pushed (Command A-Stop)
   *
   * <p>Triggering button released
   */
  @Override
  public boolean isFinished() {

    // if we cannot see any tags and too many graceframes have elapsed then cancel
    // along with stopping rumble is possible
    if (!visionSubsystem.CanSeeTagUnchanging(TagAligningToo)
        && conditionalDriveCommand != null
        && GraceFrames > SmartDashboard.getNumber("MaxGraceFrames", 10)) {
      conditionalDriveCommand.cancel();
      if (driveXboxController != null) {
        driveXboxController.setRumble(RumbleType.kBothRumble, 0);
      }
      Logger.getGlobal().log(Level.WARNING, "Cannot see tag exit");
      return true;
    }

    // if the back buttons have been pressed then cancel
    // along with stopping rumble is possible
    if ((RobotContainer.driverXbox.back().getAsBoolean()
            || RobotContainer.coralController.back().getAsBoolean())
        && conditionalDriveCommand != null) {
      conditionalDriveCommand.cancel();
      if (driveXboxController != null) {
        driveXboxController.setRumble(RumbleType.kBothRumble, 0);
      }
      Logger.getGlobal().log(Level.WARNING, "Back button exit");
      return true;
    }

    // if the triggering button has been released then cancel
    // along with stopping rumble is possible
    if (trigger != null) {
      if (!trigger.getAsBoolean() && conditionalDriveCommand != null) {
        conditionalDriveCommand.cancel();
        if (driveXboxController != null) {
          driveXboxController.setRumble(RumbleType.kBothRumble, 0);
        }
        Logger.getGlobal().log(Level.WARNING, "Button release exit");
        return true;
      }
    }

    // if we are too close on the X, Y, and rot then cancel
    // along with stopping rumble is possible
    if (TagLocation != null) {

      if ((TagLocation.getY()
                  < ((-SmartDashboard.getNumber("Y-Stop-Dist", 0.025))
                      + SmartDashboard.getNumber("yError", 0.1))
              && TagLocation.getY()
                  > ((-SmartDashboard.getNumber("Y-Stop-Dist", 0.025))
                      - SmartDashboard.getNumber("yError", 0.1)))
          && conditionalDriveCommand != null) {
        Logger.getGlobal().log(Level.INFO, "Y Satisfied");
        if ((TagLocation.getX()
                < ((-SmartDashboard.getNumber("X-Stop-Dist", 0.025))
                    + SmartDashboard.getNumber("xError", 0.1))
            && TagLocation.getX()
                > ((-SmartDashboard.getNumber("X-Stop-Dist", 0.025))
                    - SmartDashboard.getNumber("xError", 0.1)))) {
          Logger.getGlobal().log(Level.INFO, "Y + X Satisfied");
          if ((TagLocation.getRotation().getDegrees() < SmartDashboard.getNumber("rError", 1))
              && (TagLocation.getRotation().getDegrees()
                  > -SmartDashboard.getNumber("rError", 1))) {
            conditionalDriveCommand.cancel();
            if (driveXboxController != null) {
              driveXboxController.setRumble(RumbleType.kBothRumble, 0);
            }
            Logger.getGlobal().log(Level.WARNING, "Y, X, Rot Satisfied");
            return true;
          }
        }
      }
    }

    return false;
  }
}
