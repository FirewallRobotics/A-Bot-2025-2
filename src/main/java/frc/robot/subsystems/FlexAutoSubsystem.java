package frc.robot.subsystems;

import java.util.logging.Level;
import java.util.logging.Logger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.AlgaeIntakeCommand;
import frc.robot.commands.AlgaeShootCommand;
import frc.robot.commands.AlignWithNearest;
import frc.robot.commands.ArmLevel2;
import frc.robot.commands.ArmLevel3;
import frc.robot.commands.CoralIntakeCommand;
import frc.robot.commands.CoralShootCommand;
import frc.robot.commands.ElevatorMoveLevel2;
import frc.robot.commands.ElevatorMoveLevel3;
import frc.robot.commands.GoToCommand;
import frc.robot.commands.WristToL1;
import frc.robot.commands.algaeStopIntake;

public class FlexAutoSubsystem extends SubsystemBase {

  // create a counter as a last resort override
  private int counter = 0;

  // Create a variable for the stage of auto we are in
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

  private int stage = 0;

  public int getStage() {
    return stage;
  }

  // After max ticks we give up on the action and goto a "PANIC" location
  private int maxTicksBeforePanic = 500;

  private Pose2d PANICPose2d;

  /**
   * Auto that scores L2 -> remove algae -> get coral -> scores L3 -> goes to the PANIC point
   * @param maxTicksBeforePanic Ticks till Flex panics
   * @param PANICPose2d Location for Flex to goto when in the midst of a panic
   */
  public FlexAutoSubsystem(int maxTicksBeforePanic, Pose2d PANICPose2d) {
    this.maxTicksBeforePanic = maxTicksBeforePanic;
    this.PANICPose2d = PANICPose2d;
  }

  public boolean CHECKPANIC(){
    if(counter >= maxTicksBeforePanic){
      Logger.getGlobal().log(Level.WARNING, "OMG ITS HAPPENING AHHHHHHHHHH! GOING TO PANIC POINT!");
      RobotContainer.drivebase.driveToPose(PANICPose2d).schedule();
      return true;
    }
    return false;
  }

  /**
   * Check to see if a new path is available
   *
   * @return If we are not moving and the cycle counter is high enough
   */
  public boolean isNewPathAvailable() {
    switch (stage) {
      case 0:
        if(CHECKPANIC()){
          return true;
        }
        return moveCommand.isFinished();
      case 1:
        if(CHECKPANIC()){
          return true;
        }
        return sequentialMoveCommand.isFinished();
      case 2:
        if(CHECKPANIC()){
          return true;
        }
        return sequentialMoveCommand.isFinished();
      case 3:
        if(CHECKPANIC()){
          return true;
        }
        return sequentialMoveCommand.isFinished();
      case 4:
        if(CHECKPANIC()){
          return true;
        }
        return moveCommand.isFinished();
      case 5:
        if(CHECKPANIC()){
          return true;
        }
        return sequentialMoveCommand.isFinished();
      case 6:
        if(CHECKPANIC()){
          return true;
        }
        return moveCommand.isFinished();
      case 7:
        if(CHECKPANIC()){
          return true;
        }
        return sequentialMoveCommand.isFinished();
      case 8:
        if(CHECKPANIC()){
          return true;
        }
        return moveCommand.isFinished();
      case 9:
        if(CHECKPANIC()){
          return true;
        }
        return sequentialMoveCommand.isFinished();
      case 10:
        if(CHECKPANIC()){
          return true;
        }
        return sequentialMoveCommand.isFinished();
    }
    // failsafe end the command
    return true;
  }

  Command moveCommand;
  SequentialCommandGroup sequentialMoveCommand;

  /**
   * Creates and follows a path based on what we can see with vision goals: Auto can do 2 things 1)
   * drop off preloaded coral and do cycles between coral station and reef 2) drop off preloaded
   * coral and do cycles between reef(algae) and processor
   *
   * @param CoralStationChoose The current prefered Coral Station (We will cycle to and from this)
   */
  public void CreatePath(String CoralStationChoose) {
    switch (stage) {
        // move to reef (pathplanner)
      case 0:
        moveCommand = new GoToCommand(6);
        moveCommand.schedule();
        break;
      case 1:
        sequentialMoveCommand =
            new SequentialCommandGroup(
                new ElevatorMoveLevel2(RobotContainer.elevatorSubsystem),
                // TODO: Placeholder for level wrist command
                new ArmLevel2(RobotContainer.coralHoldAngleSubsystem),
                new AlignWithNearest(-0.15, null));
        sequentialMoveCommand.schedule();
        break;
      case 2:
        sequentialMoveCommand =
            new SequentialCommandGroup(
                new ArmLevel2(RobotContainer.coralHoldAngleSubsystem),
                new CoralShootCommand(RobotContainer.coralHoldSubsystem));
        sequentialMoveCommand.schedule();
        break;
      case 3:
        sequentialMoveCommand =
            new SequentialCommandGroup(
                RobotContainer.drivebase.driveToDistanceCommand(-0.25, 0),
                // TODO: Switch with level command for elevator and arm getting Algae between L2 and
                // L3
                new ParallelCommandGroup(
                    new ElevatorMoveLevel3(RobotContainer.elevatorSubsystem),
                    new ArmLevel3(RobotContainer.coralHoldAngleSubsystem)),
                new AlignWithNearest(0, null));
        sequentialMoveCommand.schedule();
        break;
      case 4:
        moveCommand = new AlgaeIntakeCommand(RobotContainer.algaeSubsystem);
        moveCommand.schedule();
        break;
      case 5:
        Command rotCommand = RobotContainer.drivebase.driveCommand(() -> 0, () -> 0, () -> 2);
        sequentialMoveCommand =
            new SequentialCommandGroup(
                rotCommand,
                new WaitCommand(0.5),
                run(
                    () -> {
                      rotCommand.cancel();
                    }),
                new AlgaeShootCommand(RobotContainer.algaeSubsystem),
                new WaitCommand(0.25),
                new algaeStopIntake(RobotContainer.algaeSubsystem));
        sequentialMoveCommand.schedule();
        break;
      case 6:
        moveCommand = RobotContainer.getCoralPathCommand(CoralStationChoose);
        moveCommand.schedule();
        break;
      case 7:
        sequentialMoveCommand = new SequentialCommandGroup(
          new WristToL1(RobotContainer.coralHoldAngleSubsystem),
          new CoralIntakeCommand(RobotContainer.coralHoldSubsystem));
        sequentialMoveCommand.schedule();
        break;
      case 8:
        moveCommand = new GoToCommand(6);
        moveCommand.schedule();
        break;
      case 9:
        sequentialMoveCommand =
        new SequentialCommandGroup(
          new ElevatorMoveLevel3(RobotContainer.elevatorSubsystem),
          // TODO: Placeholder for level wrist command
          new ArmLevel2(RobotContainer.coralHoldAngleSubsystem),
          new AlignWithNearest(-0.15, null));
        sequentialMoveCommand.schedule();
        break;
      case 10:
        sequentialMoveCommand =
        new SequentialCommandGroup(
          new ArmLevel2(RobotContainer.coralHoldAngleSubsystem),
          new CoralShootCommand(RobotContainer.coralHoldSubsystem));
        sequentialMoveCommand.schedule();
        break;
    }
  }
}
