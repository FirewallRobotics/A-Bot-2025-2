package frc.robot.subsystems;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.LimelightHelpers.LimelightTarget_Retro;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commands.AlgaeIntakeCommand;
import frc.robot.commands.AlgaeShootCommand;
import frc.robot.commands.AlignWithNearest;
import frc.robot.commands.CoralIntakeCommand;
import frc.robot.commands.CoralShootCommand;
import java.util.List;
import java.util.Optional;
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

  int elevatorSearch = 0;

  public void CreatePath(PathConstraints constraints) {

    // goals: Auto can do 2 things
    // 1) drop off preloaded coral and do cycles between coral station and reef
    // 2) drop off preloaded coral and do cycles between reef(algae) and processor

    // if our plan is to go to the coral station then were doing coral station cycles
    if (SmartDashboard.getBoolean("AutoThenGoToCoralStation", false)) {

      // if a new path is available AKA we are not moving
      if (isNewPathAvailable()) {

        // get the reef location
        Translation2d temp2 = getReefLocationInFieldSpace();
        if (temp2 == null) {
          return;
        }

        // add driving to the reef and dropping a coral to the command stack
        Robot.autonomousCommand.addCommands(
            RobotContainer.drivebase.driveToPose(new Pose2d(temp2.getX(), temp2.getY(), null)),
            new CoralShootCommand(RobotContainer.coralHoldSubsystem));

        // get the coral station location
        temp2 = getCoralStationLocationInFieldSpace();
        if (temp2 == null) {
          return;
        }

        // add driving to the coral station and getting a coral to the stack then schedule all
        // commands to run
        Robot.autonomousCommand.addCommands(
            RobotContainer.drivebase.driveToPose(
                new Pose2d(temp2.getX(), temp2.getY(), new Rotation2d(135))),
            new CoralIntakeCommand(RobotContainer.coralHoldSubsystem));
        Robot.autonomousCommand.schedule();
      }
      // if we are not going to go to the coral station then we are doing algae cycles
    } else {

      // if a new path is available AKA we are not moving
      if (isNewPathAvailable()) {

        // The algae we can take is on the reef and placed randomly
        // if we can see an Algae then
        if (VisionSubsystem.CanSeeAlgae()) {

          // grab an algae and wait a second to the commands stack
          Robot.autonomousCommand.addCommands(
              new AlgaeIntakeCommand(RobotContainer.algaeSubsystem), new WaitCommand(1));

          // get our Alliance
          Optional<Alliance> ally = DriverStation.getAlliance();
          if (ally.get() == Alliance.Blue) {
            // drive to blue processor if blue
            Robot.autonomousCommand.addCommands(
                RobotContainer.drivebase.driveToPose(AlignWithNearest.TagPos[3]));
          } else {
            // drive to red processor if red
            Robot.autonomousCommand.addCommands(
                RobotContainer.drivebase.driveToPose(AlignWithNearest.TagPos[16]));
          }

          // then shoot that algae and wait a second
          // finally schedule the thing
          Robot.autonomousCommand.addCommands(
              new AlgaeShootCommand(RobotContainer.algaeSubsystem), new WaitCommand(1));
          Robot.autonomousCommand.schedule();

          // wait for the command to finish running
          while (!Robot.autonomousCommand.isFinished()) {
            System.out.println("Running our Course");
          }

          // turn around to find the reef
          RobotContainer.drivebase.drive(new Translation2d(0, 0), 180, false);

          // find the reef
          Translation2d temp2 = getReefLocationInFieldSpace();

          // if we can't then wait till we can
          while (temp2 == null) {
            temp2 = getReefLocationInFieldSpace();
          }

          // drive to the reef if we can see it
          Robot.autonomousCommand =
              new SequentialCommandGroup(
                  RobotContainer.drivebase.driveToPose(
                      new Pose2d(temp2.getX(), temp2.getY(), new Rotation2d(135))));

          // if we can't see an algae
        } else {

          // move the elevator
          RobotContainer.elevatorSubsystem.setLevel(elevatorSearch);

          // wait for its movement
          while (!RobotContainer.elevatorSubsystem.isFinished(elevatorSearch)) {
            System.out.println("Waiting for elevator");
          }

          // move the elevator up if we are not at the top of its reach
          if (elevatorSearch != ElevatorSubsystem.levels.length) {
            elevatorSearch++;

            // if the elevator is at the top then move to the next reef location to continue search
          } else {
            while (VisionSubsystem.DistanceToReef() == -1) {
              // move backwards till we can see tags
              RobotContainer.drivebase.drive(new Translation2d(-1, 0), 0, false);
            }

            // get that tags ID and add 1 to it to find the next
            int tag = VisionSubsystem.getTags()[0] + 1;

            // to prevent an overflow sending us to narnia or an error
            if (tag == 23) {
              tag = 17;
            }
            if (tag == 12) {
              tag = 6;
            }

            // then move to the corresponding position given to us via the grace of me hardcoding
            // values into Drive Assistance
            // also schedule the entire movement
            Robot.autonomousCommand.addCommands(
                RobotContainer.drivebase.driveToPose(AlignWithNearest.TagPos[tag]));
            Robot.autonomousCommand.schedule();
            // wait for movement to be done
            while (!Robot.autonomousCommand.isFinished()) {
              System.out.println("Waiting for movement");
            }
            elevatorSearch = 0;
          }
        }
      }
    }
  }
}
