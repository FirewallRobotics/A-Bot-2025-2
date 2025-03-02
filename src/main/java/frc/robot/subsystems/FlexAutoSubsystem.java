package frc.robot.subsystems;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers.LimelightTarget_Retro;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commands.CoralIntakeCommand;
import frc.robot.commands.CoralShootCommand;
import java.util.List;
import java.util.function.DoubleSupplier;

public class FlexAutoSubsystem extends SubsystemBase {

  List<Pose2d> returnPose2ds;
  double[] RobotSpaceCoralLocation;
  double[] RobotSpaceAlgaeLocation;
  double[] ReefLocation;
  double[] ProcLocation;
  Pose2d RobotFieldSpace;

  // create a selector for flex auto modes
  private final SendableChooser<String> m_AutoObjChooser = new SendableChooser<>();

  public FlexAutoSubsystem() {
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

  public boolean isNewPathAvailable() {
    // new path is avaliable if we are not moving
    if (RobotContainer.drivebase.getRobotVelocity().vxMetersPerSecond < 0.2
        && RobotContainer.drivebase.getRobotVelocity().vyMetersPerSecond < 0.2) {
      return true;
    }
    return false;
  }

  public Translation2d getReefLocationInFieldSpace() {

    // get reef location in robot space
    ReefLocation = VisionSubsystem.getReefLocation();

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

  public void CreatePath(PathConstraints constraints) {

    // goals: Auto can do 2 things
    // 1) drop off preloaded coral and do cycles between coral station and reef
    // 2) drop off preloaded coral and do cycles between reef(algae) and processor

    // if our plan is to go to the coral station then were doing coral station cycles
    if (SmartDashboard.getBoolean("AutoThenGoToCoralStation", false)) {

      // if a new path is available AKA we are not moving
      if (isNewPathAvailable()) {

        Translation2d temp2 = getReefLocationInFieldSpace();
        if (temp2 == null) {
          return;
        }
        Robot.autonomousCommand.andThen(
            RobotContainer.drivebase.driveToPose(new Pose2d(temp2.getX(), temp2.getY(), null)),
            new CoralShootCommand(RobotContainer.coralHoldSubsystem));
        temp2 = getCoralStationLocationInFieldSpace();
        if (temp2 == null) {
          return;
        }
        Robot.autonomousCommand.andThen(
            RobotContainer.drivebase.driveToPose(
                new Pose2d(temp2.getX(), temp2.getY(), new Rotation2d(135))),
            new CoralIntakeCommand(RobotContainer.coralHoldSubsystem));
      }
      // if we are not going to go to the coral station then we are doing algae cycles
    } else {

      // TODO: make this not need an external drive command

      // if a new path is available AKA we are not moving
      if (isNewPathAvailable()) {

        // get our current location in field space
        Pose3d temp = VisionSubsystem.getRobotPoseInFieldSpace();

        // set the start position to our location
        returnPose2ds.add(new Pose2d(temp.getX(), temp.getY(), null));

        // go to the reef
        Translation2d temp2 = getReefLocationInFieldSpace();
        returnPose2ds.add(new Pose2d(temp2.getX(), temp2.getY(), null));

        // set our goal to the processor
        Translation2d temp3 = getProcessorLocationInFieldSpace();
        returnPose2ds.add(new Pose2d(temp3.getX(), temp3.getY(), null));
      }
    }
  }
}
