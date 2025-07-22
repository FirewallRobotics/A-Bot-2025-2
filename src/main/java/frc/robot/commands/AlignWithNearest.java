package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
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
  PIDController rController;
  PIDController xController;
  PIDController yController;

  int TagAligningToo;
  ParallelRaceGroup conditionalDriveCommand;
  Pose2d TagLocation;
  int GraceFrames;
  VisionSubsystem visionSubsystem;

  // add vision as a requirement to run
  public AlignWithNearest() {
    // P = speed
    // I = smoothing
    // D = time
    xController = new PIDController(1.14, 0, 0);
    yController = new PIDController(1.14, 0, 0);
    rController = new PIDController(0.27, 0, 0.2);
    TagLocation = new Pose2d(0, 0, new Rotation2d(0));
    visionSubsystem = new VisionSubsystem();
  }

  @Override
  public void initialize() {
    xController.setSetpoint(0);
    xController.setTolerance(1);
    yController.setSetpoint(0);
    yController.setTolerance(1);
    rController.setSetpoint(0);
    rController.setTolerance(1);
  }

  @Override
  public void execute() {
    int[] tags = visionSubsystem.getTagsUnchanging();
    // False means it's not null
    // True means that it is null
    boolean isNullDoubleTest = false;

    if (tags.length > 0) {
      TagAligningToo = tags[0];
      // Provides direct connection that bypasses pathplanner (more accurate)
      // If we use this, put it in execute and have it change constantly to update as data streams
      // in.
      TagLocation = visionSubsystem.getTagPose2dUnchanging(TagAligningToo);

      // TagLocation != null
      //    && TagLocation.getTranslation() != null
      //    && TagLocation.getRotation() != null

      if (isNullDoubleTest == false) {

        Command driveCommand;

        SmartDashboard.putNumberArray(
            "Calculated pos",
            new double[] {
              TagLocation.getY(),
              yController.calculate(TagLocation.getY()),
              TagLocation.getX(),
              xController.calculate(TagLocation.getX()),
              MathUtil.inputModulus(TagLocation.getRotation().getDegrees(), -180, 180),
              rController.calculate(
                  MathUtil.inputModulus(TagLocation.getRotation().getDegrees(), -180, 180))
            });

        driveCommand =
            RobotContainer.drivebase.driveCommand(
                new ChassisSpeeds(
                    (yController.calculate(TagLocation.getY())),
                    (xController.calculate(TagLocation.getX())),
                    (rController.calculate(
                        MathUtil.inputModulus(
                            TagLocation.getRotation().getDegrees(), -180, 180)))));

        conditionalDriveCommand =
            driveCommand.until(
                () ->
                    Math.abs(TagLocation.getY()) < SmartDashboard.getNumber("Y-Stop-Dist", 0.025)
                        || !visionSubsystem.CanSeeTagUnchanging(TagAligningToo)
                        || (RobotContainer.driverXbox.back().getAsBoolean()
                            || RobotContainer.coralController.back().getAsBoolean())
                        || !RobotContainer.coralController.rightBumper().getAsBoolean());

        if (Math.abs(TagLocation.getY()) > SmartDashboard.getNumber("Y-Stop-Dist", 0.025)) {

          // Logger.getGlobal().log(Level.INFO, "Too far");

          if (visionSubsystem.CanSeeTagUnchanging(TagAligningToo)) {
            GraceFrames = 0;

            // Logger.getGlobal().log(Level.INFO, "Can see tag");

            if ((!RobotContainer.driverXbox.back().getAsBoolean()
                && !RobotContainer.coralController.back().getAsBoolean())) {

              // Logger.getGlobal().log(Level.INFO, "No back buttons");

              if (RobotContainer.coralController.rightBumper().getAsBoolean()) {

                conditionalDriveCommand.schedule();
                // Logger.getGlobal()
                //    .log(
                //        Level.INFO,
                //        "Move too: " + -(TagLocation.getY()) + " " + -(TagLocation.getX()));
              } else {
                Logger.getGlobal().log(Level.WARNING, "Button release cancel");
                if (conditionalDriveCommand != null) {
                  conditionalDriveCommand.cancel();
                }
              }
            } else {
              Logger.getGlobal().log(Level.WARNING, "Back button cancel");
              if (conditionalDriveCommand != null) {
                conditionalDriveCommand.cancel();
              }
            }
          } else {
            Logger.getGlobal().log(Level.WARNING, "Cannot see tag cancel");
            if (conditionalDriveCommand != null) {
              conditionalDriveCommand.cancel();
            }
          }
        } else {
          Logger.getGlobal().log(Level.WARNING, "Too close cancel");
          if (conditionalDriveCommand != null) {
            conditionalDriveCommand.cancel();
          }
        }

        // Logger.getGlobal()
        //     .log(Level.WARNING, "CanSee: " +
        // visionSubsystem.CanSeeTagUnchanging(TagAligningToo));
        // Logger.getGlobal()
        //    .log(
        //        Level.WARNING,
        //        "TooFar: "
        //            + (Math.abs(TagLocation.getX())
        //                > SmartDashboard.getNumber("Y-Stop-Dist", 0.025)));
        // Logger.getGlobal().log(Level.WARNING, "DistValue: " + (Math.abs(TagLocation.getX())));
      } else {
        Logger.getGlobal().log(Level.WARNING, "Null location cancel");
        if (conditionalDriveCommand != null) {
          conditionalDriveCommand.cancel();
        }
      }
    } else if (GraceFrames > 10) {
      Logger.getGlobal().log(Level.WARNING, "No tags cancel");
      if (conditionalDriveCommand != null) {
        conditionalDriveCommand.cancel();
      }
    } else {
      GraceFrames += 1;
    }
  }

  @Override
  public boolean isFinished() {

    if (!visionSubsystem.CanSeeTagUnchanging(TagAligningToo)
        && conditionalDriveCommand != null
        && GraceFrames > 10) {
      conditionalDriveCommand.cancel();
      Logger.getGlobal().log(Level.WARNING, "Cannot see tag exit");
      return true;
    }

    if ((RobotContainer.driverXbox.back().getAsBoolean()
            || RobotContainer.coralController.back().getAsBoolean())
        && conditionalDriveCommand != null) {
      conditionalDriveCommand.cancel();
      Logger.getGlobal().log(Level.WARNING, "Back button exit");
      return true;
    }

    if (!RobotContainer.coralController.rightBumper().getAsBoolean()
        && conditionalDriveCommand != null) {
      conditionalDriveCommand.cancel();
      Logger.getGlobal().log(Level.WARNING, "Button release exit");
      return true;
    }

    if (TagLocation != null) {
      if (Math.abs(TagLocation.getY()) < SmartDashboard.getNumber("Y-Stop-Dist", 0.025)
          && conditionalDriveCommand != null) {
        conditionalDriveCommand.cancel();
        Logger.getGlobal().log(Level.WARNING, "Too close exit");
        return true;
      }
    } else if (conditionalDriveCommand != null) {
      conditionalDriveCommand.cancel();
      Logger.getGlobal().log(Level.WARNING, "Cannot get position exit");
      return true;
    }

    return false;
  }
}
