package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers.LimelightTarget_Retro;
import frc.robot.RobotContainer;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.logging.Level;
import java.util.logging.Logger;

public class FlexAutoSubsystem extends SubsystemBase {

  List<Pose2d> returnPose2ds;
  double[] RobotSpaceCoralLocation;
  double[] RobotSpaceAlgaeLocation;
  double[] ReefLocation;
  double[] ProcLocation;
  Pose2d RobotFieldSpace;

  // create a selector for flex auto modes
  private final SendableChooser<String> m_AutoObjChooser = new SendableChooser<>();
  private int counter = 0;

  public FlexAutoSubsystem() {
    SmartDashboard.putNumber("FlexCounter", counter);
    // add selections for flex auto to smartdashboard
    m_AutoObjChooser.setDefaultOption("Coral", "coral");
    m_AutoObjChooser.addOption("Algae", "algae");
    SmartDashboard.putData("Auto Obj choices", m_AutoObjChooser);
    SmartDashboard.putNumber("AutoScanSpeed", 1.0);
    SmartDashboard.putNumber("Ultrasonics Coral", 50);
    SmartDashboard.putNumber("Ultrasonics Algae", 50);
    SmartDashboard.putNumber("AutoMoveSpeed", 1.0);
    SmartDashboard.putNumber("AutoRotateSpeed", 1.0);
  }

  /**
   * Check to see if a new path is available based on if we are not moving and flex has had enough
   * time to cycle
   *
   * @return If we are not moving and the cycle counter is high enough
   */
  public boolean isNewPathAvailable() {
    // new path is avaliable if we are not moving
    if (RobotContainer.drivebase.getRobotVelocity().vxMetersPerSecond < 0.05
        && RobotContainer.drivebase.getRobotVelocity().vyMetersPerSecond < 0.05
        && counter >= 400) {
      counter = 0;
      return true;
    }
    SmartDashboard.putNumber("FlexCounter", counter);
    counter += 1;
    return false;
  }

  /**
   * Gets the location of a reef tag in field space
   *
   * @return The location of the nearest reef tag in field space
   */
  public Translation2d getReefLocationInFieldSpace() {

    // get reef location in robot space
    ReefLocation = VisionSubsystem.getReefLocation();

    // if we dont have the reefs location find it by spinning slowly
    if (ReefLocation[0] == -1 && ReefLocation[1] == -1) {
      DoubleSupplier scanspeed = () -> SmartDashboard.getNumber("AutoScanSpeed", 1.0);
      RobotContainer.drivebase.driveCommand(() -> 0, () -> 0, scanspeed).schedule();
    } else {
      // if we do have the reefs location then convert it
      // first zero the drivecommand so the math stays right
      RobotContainer.drivebase.driveCommand(() -> 0, () -> 0, () -> 0).schedule();

      // get the robots location in field space
      RobotFieldSpace = LimelightTarget_Retro.getRobotPose_FieldSpace2D();

      // do the math to find the location of the reef by adding together the values
      double xActual = ReefLocation[0] + RobotFieldSpace.getX();
      double yActual = ReefLocation[1] + RobotFieldSpace.getY();

      // return the values
      return new Translation2d(xActual, yActual);
    }
    return null;
  }

  /**
   * Gets the location of a Coral Station tag in field space
   *
   * @return The location of the nearest Coral Station tag in field space
   */
  public Translation2d getCoralStationLocationInFieldSpace() {

    // get reef location in robot space
    ReefLocation = VisionSubsystem.getCoralStationLocation();

    // if we dont have the reefs location find it by spinning slowly
    if (ReefLocation[0] == -1 && ReefLocation[1] == -1) {
      DoubleSupplier scanspeed = () -> SmartDashboard.getNumber("AutoScanSpeed", 1.0);
      RobotContainer.drivebase.driveCommand(() -> 0, () -> 0, scanspeed);
    } else {
      // if we do have the reefs location then convert it
      // first zero the drivecommand so the math stays right
      RobotContainer.drivebase.driveCommand(() -> 0, () -> 0, () -> 0);

      // get the robots location in field space
      RobotFieldSpace = LimelightTarget_Retro.getRobotPose_FieldSpace2D();

      // do the math to find the location of the reef by adding together the values
      double xActual = ReefLocation[0] + RobotFieldSpace.getX();
      double yActual = ReefLocation[1] + RobotFieldSpace.getY();

      // return the values
      return new Translation2d(xActual, yActual);
    }
    return null;
  }

  /**
   * Gets the location of a Processor tag in field space
   *
   * @return The location of the nearest Processor tag in field space
   */
  public Translation2d getProcessorLocationInFieldSpace() {

    // get Processor location in robot space
    ProcLocation = VisionSubsystem.getProcessorLocation();

    // if we dont have its location find it by spinning slowly
    if (ProcLocation[0] == -1 && ProcLocation[1] == -1) {
      DoubleSupplier scanspeed = () -> SmartDashboard.getNumber("AutoScanSpeed", 1.0);
      RobotContainer.drivebase.driveCommand(null, null, scanspeed);
    } else {
      // if we do have its location then convert it
      // first zero the drivecommand so the math stays right
      RobotContainer.drivebase.driveCommand(null, null, null);

      // get the robots location in field space
      RobotFieldSpace = LimelightTarget_Retro.getRobotPose_FieldSpace2D();

      // do the math to find the location of it by adding together the values
      double xActual = ProcLocation[0] + RobotFieldSpace.getX();
      double yActual = ProcLocation[1] + RobotFieldSpace.getY();

      // return the values
      return new Translation2d(xActual, yActual);
    }
    return null;
  }

  public void CreatePath() {
    if (VisionSubsystem.getTagPose2d().getX() >= 0.3) {
      Logger.getGlobal().log(Level.INFO, "Turning ClockWise");
      RobotContainer.drivebase.drive(new Translation2d(0, 0), -5, false);
    } else if (VisionSubsystem.getTagPose2d().getX() <= -0.3) {
      Logger.getGlobal().log(Level.INFO, "Turning CCW");
      RobotContainer.drivebase.drive(new Translation2d(0, 0), 5, false);
    } else {
      Logger.getGlobal().log(Level.INFO, "Stopping");
      RobotContainer.drivebase.drive(new Translation2d(0, 0), 0, false);
    }
  }
}
