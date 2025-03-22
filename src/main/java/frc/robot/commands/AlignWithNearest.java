package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers.LimelightTarget_Retro;
import frc.robot.RobotContainer;
import frc.robot.subsystems.VisionSubsystem;

import java.util.function.DoubleSupplier;
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

  public Pose2d getReefLocationInFieldSpace() {

    // get reef location in robot space
    Pose2d ReefLocation = VisionSubsystem.getReefLocationPose2d();

    // if we dont have the reefs location find it by spinning slowly
    if (ReefLocation.getX() == -1 && ReefLocation.getY() == -1) {
      DoubleSupplier scanspeed = () -> SmartDashboard.getNumber("AutoScanSpeed", 1.0);
      RobotContainer.drivebase.driveCommand(() -> 0, () -> 0, scanspeed).schedule();
    } else {
      // if we do have the reefs location then convert it
      // first zero the drivecommand so the math stays right
      RobotContainer.drivebase.driveCommand(() -> 0, () -> 0, () -> 0).schedule();

      // get the robots location in field space
      Pose2d RobotFieldSpace = RobotContainer.drivebase.getPose2d();

      // do the math to find the location of the reef by adding together the values
      double xActual = ReefLocation.getX() + RobotFieldSpace.getX();
      double yActual = ReefLocation.getY() + RobotFieldSpace.getY();

      // return the values
      return new Pose2d(new Translation2d(xActual, yActual), ReefLocation.getRotation());
    }
    return new Pose2d(new Translation2d(-1, -1), ReefLocation.getRotation());
  }

  // add vision as a requirement to run
  public AlignWithNearest() {}

  @Override
  public void execute() {
    if (getReefLocationInFieldSpace().getTranslation().getX() != -1) {

      if(VisionSubsystem.getReefLocation()[0] >= 0.1 || VisionSubsystem.getReefLocation()[0] <= -0.1){
        Logger.getGlobal().log(Level.INFO, "Driver Assist Going Left/Right");
        RobotContainer.drivebase.drive(new Translation2d(VisionSubsystem.getReefLocation()[0], 0), 0, false);
      }else{
        Logger.getGlobal().log(Level.INFO, "Driver Assist Going FWD To Reef");
        RobotContainer.drivebase.driveToDistanceCommand(VisionSubsystem.DistanceToReef(), 1.5);
      }
    } else {

      // log that we cannot see anything to goto
      Logger.getGlobal().log(Level.WARNING, "Driver Assist Cannot Find A Valid Target!");
    }
  }

  @Override
  public boolean isFinished() {
    return VisionSubsystem.DistanceToReef() == -1;
  }
}
