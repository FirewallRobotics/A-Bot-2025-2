package frc.robot.subsystems;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.*;
import java.util.List;
import java.util.Optional;

public class FlexAutoSubsystem extends SubsystemBase {

  List<Pose2d> returnPose2ds;
  double[] RobotSpaceCoralLocation;
  double[] RobotSpaceAlgaeLocation;
  double[] ReefLocation;
  double[] ProcLocation;
  Pose2d RobotFieldSpace;

  // create a selector for flex auto modes
  private int counter = 0;

  public FlexAutoSubsystem() {}

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

  int elevatorSearch = 0;

  /**
   * Creates and follows a path based on what we can see with vision goals: Auto can do 2 things 1)
   * drop off preloaded coral and do cycles between coral station and reef 2) drop off preloaded
   * coral and do cycles between reef(algae) and processor
   *
   * @param constraints The Constraints of the robot
   * @param CoralStationChoose The current prefered Coral Station (We will cycle to and from this)
   */
  @SuppressWarnings("static-access")
  public void CreatePath(PathConstraints constraints, String CoralStationChoose) {

    // if our plan is to go to the coral station then were doing coral station cycles
    if (!CoralStationChoose.equals("stop")) {

      Optional<Alliance> ally = DriverStation.getAlliance();
      if (ally.get() == Alliance.Blue) {
        new SequentialCommandGroup(
                new ElevatorMoveLevel1(RobotContainer.elevatorSubsystem),
                new WristToL1(RobotContainer.coralHoldAngleSubsystem),
                new WristUp(RobotContainer.coralHoldAngleSubsystem),
                new WaitCommand(0.05),
                new WristStop(RobotContainer.coralHoldAngleSubsystem),
                new CoralIntakeCommand(RobotContainer.coralHoldSubsystem),
                new WaitCommand(2),
                new RobotContainer()
                    .drivebase.driveToPose(
                        new Pose2d(3.7, 5.4, new Rotation2d(Math.toRadians(-60)))),
                new WristToL1(RobotContainer.coralHoldAngleSubsystem),
                new CoralShootCommand(RobotContainer.coralHoldSubsystem),
                new WaitCommand(1),
                new stopCoralIntake(RobotContainer.coralHoldSubsystem),
                new RobotContainer()
                    .drivebase.driveToPose(
                        new Pose2d(1.5, 6.85, new Rotation2d(Math.toRadians(124)))))
            .schedule();
      } else {
        new SequentialCommandGroup(
                new ElevatorMoveLevel1(RobotContainer.elevatorSubsystem),
                new WristToL1(RobotContainer.coralHoldAngleSubsystem),
                new WristUp(RobotContainer.coralHoldAngleSubsystem),
                new WaitCommand(0.05),
                new WristStop(RobotContainer.coralHoldAngleSubsystem),
                new CoralIntakeCommand(RobotContainer.coralHoldSubsystem),
                new WaitCommand(2),
                new RobotContainer()
                    .drivebase.driveToPose(
                        new Pose2d(3.5, 5.4, new Rotation2d(Math.toRadians(50)))),
                new WristToL1(RobotContainer.coralHoldAngleSubsystem),
                new CoralShootCommand(RobotContainer.coralHoldSubsystem),
                new WaitCommand(1),
                new stopCoralIntake(RobotContainer.coralHoldSubsystem),
                new RobotContainer()
                    .drivebase.driveToPose(
                        new Pose2d(14, 5.4, new Rotation2d(Math.toRadians(-122)))))
            .schedule();
      }
    }
  }
}
