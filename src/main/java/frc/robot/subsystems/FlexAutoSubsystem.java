package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.AlignWithNearest;
import frc.robot.commands.ArmSetToCoralAccept;
import frc.robot.commands.ArmSetToMiddle;
import frc.robot.commands.CoralIntakeCommand;
import frc.robot.commands.CoralShootCommand;
import frc.robot.commands.ElevatorMoveLevel2;
import frc.robot.commands.ElevatorMoveLevel3;
import frc.robot.commands.GoToCommand;

public class FlexAutoSubsystem extends SubsystemBase {

  // Stages we go thru
  // 0 - move to reef (pathplanner)
  // 1 - align with reef
  // 2 - shoot (L2)
  // 3 - realign with center
  // 4 - grab algae
  // 5 - spit out
  // 6 - goto coral station
  // 7 - intake
  // 8 - move to reef (pathplanner)
  // 9 - align with reef
  // 10 - shoot(L3)

  private SequentialCommandGroup oneSequentialCommand;
  private Command rotCommand;

  /**
   * Auto that scores L2 -> remove algae -> get coral -> scores L3
   */
  public FlexAutoSubsystem() {
    rotCommand = RobotContainer.drivebase.driveCommand(() -> 0, () -> 0, () -> 2);
  }

  /**
   * Check to see if a new path is available
   *
   * @return If we are not moving and the cycle counter is high enough
   */
  public boolean isNewPathAvailable() {

    if (oneSequentialCommand != null){
      return oneSequentialCommand.isFinished() || !oneSequentialCommand.isScheduled();
    }else{
      return true;
    }
  }

  /**
   * Creates and follows a path based on what we can see with vision goals: Auto can do 2 things 1)
   * drop off preloaded coral and do cycles between coral station and reef 2) drop off preloaded
   * coral and do cycles between reef(algae) and processor
   */
  public void CreatePath() {

    oneSequentialCommand =
        new SequentialCommandGroup(
          // drive to tag 11
            RobotContainer.drivebase.driveToPose(new Pose2d(AlignWithNearest.TagPos[11 - 1][0], AlignWithNearest.TagPos[11 - 1][1], new Rotation2d(AlignWithNearest.TagPos[11 - 1][2])), 2, 2),
            // move Elevator to level 2
            new ElevatorMoveLevel2(RobotContainer.elevatorCoralSubsystem),
            // Move arm to middle pos
            new ArmSetToMiddle(RobotContainer.coralWristSubsystem),
            // align with tag
            new AlignWithNearest(-0.32, null, null),
            // shoot preloaded coral
            // new ArmSetToScore(RobotContainer.coralWristSubsystem),
            new CoralShootCommand(RobotContainer.coralHoldSubsystem), new WaitCommand(0.25),
            // move back
            RobotContainer.drivebase.driveToPose(new Pose2d(AlignWithNearest.TagPos[11 - 1][0], AlignWithNearest.TagPos[11 - 1][1], new Rotation2d(AlignWithNearest.TagPos[11 - 1][2])), 2, 2),
            // move elevator to level 3
            new ElevatorMoveLevel3(RobotContainer.elevatorCoralSubsystem),
            // move coral arm up to accept
            new ArmSetToCoralAccept(RobotContainer.coralWristSubsystem),
            // align with middle of reef
            new AlignWithNearest(0, null, null),
            // rotate to nudge algae out
            rotCommand,
            // wait 0.5 sec for rotation
            new WaitCommand(0.5),
            // stop rotating
            run(
                () -> {
                  rotCommand.cancel();
                }),

            // dream stuff under
            // if the above works we are good

            // path to coral station left
            RobotContainer.getCoralPathCommand("left"),
            // move coral arm to accept
            new ArmSetToCoralAccept(RobotContainer.coralWristSubsystem),
            // intake coral (waits for coral)
            new CoralIntakeCommand(RobotContainer.coralHoldSubsystem),
            // goto tag 6
            new GoToCommand(6),
            // move to level 3
            new ElevatorMoveLevel3(RobotContainer.elevatorCoralSubsystem),
            // Move arm to middle pos
            new ArmSetToMiddle(RobotContainer.coralWristSubsystem),
            // align with tag
            new AlignWithNearest(-0.32, null, null),
            // shoot preloaded coral
            // new ArmSetToScore(RobotContainer.coralWristSubsystem),
            new CoralShootCommand(RobotContainer.coralHoldSubsystem), new WaitCommand(0.25));

    oneSequentialCommand.schedule();
  }
}
