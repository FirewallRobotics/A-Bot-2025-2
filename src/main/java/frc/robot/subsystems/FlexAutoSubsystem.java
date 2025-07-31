package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
// import frc.robot.commands.AlgaeIntakeCommand;
import frc.robot.commands.AlgaeShootCommand;
import frc.robot.commands.AlignWithNearest;
import frc.robot.commands.ArmSetToCoralAccept;
import frc.robot.commands.ArmSetToMiddle;
import frc.robot.commands.ArmSetToScore;
import frc.robot.commands.CoralIntakeCommand;
import frc.robot.commands.CoralShootCommand;
import frc.robot.commands.ElevatorMoveLevel2;
import frc.robot.commands.ElevatorMoveLevel3;
import frc.robot.commands.GoToCommand;
import frc.robot.commands.algaeStopIntake;
import java.util.logging.Level;
import java.util.logging.Logger;

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

  private static SequentialCommandGroup oneSequentialCommand;

  // private static Command moveCommand0;
  // private static SequentialCommandGroup sequentialMoveCommand1;
  // private static SequentialCommandGroup sequentialMoveCommand2;
  // private static SequentialCommandGroup sequentialMoveCommand3;
  // private static Command moveCommand4;
  // private static SequentialCommandGroup sequentialMoveCommand5;
  // private static Command moveCommand6;
  // private static SequentialCommandGroup sequentialMoveCommand7;
  // private static Command moveCommand8;
  // private static SequentialCommandGroup sequentialMoveCommand9;
  // private static SequentialCommandGroup sequentialMoveCommand10;

  /**
   * Auto that scores L2 -> remove algae -> get coral -> scores L3 -> goes to the PANIC point
   *
   * @param maxTicksBeforePanic Ticks till Flex panics
   * @param PANICPose2d Location for Flex to goto when in the midst of a panic
   */
  public FlexAutoSubsystem(int maxTicksBeforePanic, Pose2d PANICPose2d) {
    this.maxTicksBeforePanic = maxTicksBeforePanic;
    this.PANICPose2d = PANICPose2d;

    // moveCommand0 = new GoToCommand(6);

    // sequentialMoveCommand1 =
    //     new SequentialCommandGroup(
    //         new ElevatorMoveLevel2(RobotContainer.elevatorCoralSubsystem),
    //         new ArmSetToMiddle(RobotContainer.coralWristSubsystem),
    //         new AlignWithNearest(0, null, null));

    // sequentialMoveCommand2 =
    //     new SequentialCommandGroup(
    //         new ArmSetToScore(RobotContainer.coralWristSubsystem),
    //         new CoralShootCommand(RobotContainer.coralHoldSubsystem));

    // sequentialMoveCommand3 =
    //     new SequentialCommandGroup(
    //         RobotContainer.drivebase.driveToDistanceCommand(-0.25, 0),
    //         new ParallelCommandGroup(
    //             new ElevatorMoveLevel3(RobotContainer.elevatorCoralSubsystem),
    //             new ArmSetToScore(RobotContainer.coralWristSubsystem)),
    //         new AlignWithNearest(0, null, null));

    Command rotCommand = RobotContainer.drivebase.driveCommand(() -> 0, () -> 0, () -> 2);
    // sequentialMoveCommand5 =
    //     new SequentialCommandGroup(
    //         rotCommand,
    //         new WaitCommand(0.5),
    //         run(
    //             () -> {
    //               rotCommand.cancel();
    //             }),
    //         new AlgaeShootCommand(RobotContainer.algaeSubsystem),
    //         new WaitCommand(0.25),
    //         new algaeStopIntake(RobotContainer.algaeSubsystem));

    // sequentialMoveCommand7 =
    //     new SequentialCommandGroup(
    //         new ArmSetToCoralAccept(RobotContainer.coralWristSubsystem),
    //         new CoralIntakeCommand(RobotContainer.coralHoldSubsystem));

    // sequentialMoveCommand9 =
    //     new SequentialCommandGroup(
    //         new ElevatorMoveLevel3(RobotContainer.elevatorCoralSubsystem),
    //         new ArmSetToMiddle(RobotContainer.coralWristSubsystem),
    //         new AlignWithNearest(-0.15, null, null));

    // sequentialMoveCommand10 =
    //     new SequentialCommandGroup(
    //         new ArmSetToScore(RobotContainer.coralWristSubsystem),
    //         new CoralShootCommand(RobotContainer.coralHoldSubsystem));

    oneSequentialCommand = new SequentialCommandGroup(
      RobotContainer.drivebase.driveToPose(AlignWithNearest.TagPos[6 - 1], 2, 2),
      new ElevatorMoveLevel2(RobotContainer.elevatorCoralSubsystem),
      new ArmSetToMiddle(RobotContainer.coralWristSubsystem),
      new AlignWithNearest(0, null, null),
      //new ArmSetToScore(RobotContainer.coralWristSubsystem),
      new ParallelRaceGroup(
        new CoralShootCommand(RobotContainer.coralHoldSubsystem),
        new WaitCommand(0.25)),
      RobotContainer.drivebase.driveToPose(AlignWithNearest.TagPos[6 - 1], 2, 2),
      new ElevatorMoveLevel3(RobotContainer.elevatorCoralSubsystem),
      new ArmSetToCoralAccept(RobotContainer.coralWristSubsystem),
      new AlignWithNearest(0, null, null),
      rotCommand,
      new WaitCommand(0.5),
      run(
          () -> {
            rotCommand.cancel();
          }),
      new AlgaeShootCommand(RobotContainer.algaeSubsystem),
      new WaitCommand(0.25),
      new algaeStopIntake(RobotContainer.algaeSubsystem),
      RobotContainer.getCoralPathCommand("left"),
      new ArmSetToCoralAccept(RobotContainer.coralWristSubsystem),
      new CoralIntakeCommand(RobotContainer.coralHoldSubsystem),
      new GoToCommand(6),
      new ElevatorMoveLevel3(RobotContainer.elevatorCoralSubsystem),
      new ArmSetToMiddle(RobotContainer.coralWristSubsystem),
      new AlignWithNearest(-0.15, null, null),
      new ArmSetToScore(RobotContainer.coralWristSubsystem),
      new CoralShootCommand(RobotContainer.coralHoldSubsystem));
  }

  /**
   * @deprecated needs to be updated for use as a fallback
   * @return
   */
  public boolean CHECKPANIC() {
    if (counter >= maxTicksBeforePanic) {
      Logger.getGlobal().log(Level.WARNING, "OMG ITS HAPPENING AHHHHHHHHHH! GOING TO PANIC POINT!");
      RobotContainer.drivebase.driveToPose(PANICPose2d).schedule();
      return true;
    }
    return false;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("FlexAutoStage", stage);
  }

  /**
   * Check to see if a new path is available
   *
   * @return If we are not moving and the cycle counter is high enough
   */
  public boolean isNewPathAvailable() {
    // switch (stage) {
    //   case 0:
    //     if (CHECKPANIC()) {
    //       return true;
    //     }
    //     if (moveCommand0 == null) {
    //       Logger.getGlobal().log(Level.WARNING, "Null allow on stage: 0");
    //       stage += 1;
    //       return true;
    //     }
    //     if (moveCommand0.isFinished()) {
    //       stage += 1;
    //       Logger.getGlobal().log(Level.INFO, "Finished Stage: 0");
    //       return true;
    //     }
    //     return false;
    //   case 1:
    //     if (CHECKPANIC()) {
    //       return true;
    //     }
    //     if (sequentialMoveCommand1 == null) {
    //       Logger.getGlobal().log(Level.WARNING, "Null allow on stage: 1");
    //       return true;
    //     }
    //     if (sequentialMoveCommand1.isFinished()) {
    //       stage += 1;
    //       Logger.getGlobal().log(Level.INFO, "Finished Stage: 1");
    //       return true;
    //     }
    //     return false;
    //   case 2:
    //     if (CHECKPANIC()) {
    //       return true;
    //     }
    //     if (sequentialMoveCommand2 == null) {
    //       Logger.getGlobal().log(Level.WARNING, "Null allow on stage: 2");
    //       return true;
    //     }
    //     if (sequentialMoveCommand2.isFinished()) {
    //       stage += 1;
    //       Logger.getGlobal().log(Level.INFO, "Finished Stage: 2");
    //       return true;
    //     }
    //     return false;
    //   case 3:
    //     if (CHECKPANIC()) {
    //       return true;
    //     }
    //     if (sequentialMoveCommand3 == null) {
    //       Logger.getGlobal().log(Level.WARNING, "Null allow on stage: 3");
    //       return true;
    //     }
    //     if (sequentialMoveCommand3.isFinished()) {
    //       stage += 1;
    //       Logger.getGlobal().log(Level.INFO, "Finished Stage: 3");
    //       return true;
    //     }
    //     return false;
    //   case 4:
    //     if (CHECKPANIC()) {
    //       return true;
    //     }
    //     if (moveCommand4 == null) {
    //       Logger.getGlobal().log(Level.WARNING, "Null allow on stage: 4");
    //       return true;
    //     }
    //     if (moveCommand4.isFinished()) {
    //       stage += 1;
    //       Logger.getGlobal().log(Level.INFO, "Finished Stage: 4");
    //       return true;
    //     }
    //     return false;
    //   case 5:
    //     if (CHECKPANIC()) {
    //       return true;
    //     }
    //     if (sequentialMoveCommand5 == null) {
    //       Logger.getGlobal().log(Level.WARNING, "Null allow on stage: 5");
    //       return true;
    //     }
    //     if (sequentialMoveCommand5.isFinished()) {
    //       stage += 1;
    //       Logger.getGlobal().log(Level.INFO, "Finished Stage: 5");
    //       return true;
    //     }
    //     return false;
    //   case 6:
    //     if (CHECKPANIC()) {
    //       return true;
    //     }
    //     if (moveCommand6 == null) {
    //       Logger.getGlobal().log(Level.WARNING, "Null allow on stage: 6");
    //       return true;
    //     }
    //     if (moveCommand6.isFinished()) {
    //       stage += 1;
    //       Logger.getGlobal().log(Level.INFO, "Finished Stage: 6");
    //       return true;
    //     }
    //     return false;
    //   case 7:
    //     if (CHECKPANIC()) {
    //       return true;
    //     }
    //     if (sequentialMoveCommand7 == null) {
    //       Logger.getGlobal().log(Level.WARNING, "Null allow on stage: 7");
    //       return true;
    //     }
    //     if (sequentialMoveCommand7.isFinished()) {
    //       stage += 1;
    //       Logger.getGlobal().log(Level.INFO, "Finished Stage: 7");
    //       return true;
    //     }
    //     return false;
    //   case 8:
    //     if (CHECKPANIC()) {
    //       return true;
    //     }
    //     if (moveCommand8 == null) {
    //       Logger.getGlobal().log(Level.WARNING, "Null allow on stage: 8");
    //       return true;
    //     }
    //     if (moveCommand8.isFinished()) {
    //       stage += 1;
    //       Logger.getGlobal().log(Level.INFO, "Finished Stage: 8");
    //       return true;
    //     }
    //     return false;
    //   case 9:
    //     if (CHECKPANIC()) {
    //       return true;
    //     }
    //     if (sequentialMoveCommand9 == null) {
    //       Logger.getGlobal().log(Level.WARNING, "Null allow on stage: 9");
    //       return true;
    //     }
    //     if (sequentialMoveCommand9.isFinished()) {
    //       stage += 1;
    //       Logger.getGlobal().log(Level.INFO, "Finished Stage: 9");
    //       return true;
    //     }
    //     return false;
    //   case 10:
    //     if (CHECKPANIC()) {
    //       return true;
    //     }
    //     if (sequentialMoveCommand10 == null) {
    //       Logger.getGlobal().log(Level.WARNING, "Null allow on stage: 10");
    //       return true;
    //     }
    //     if (sequentialMoveCommand10.isFinished()) {
    //       stage += 1;
    //       Logger.getGlobal().log(Level.INFO, "Finished Stage: 10");
    //       return true;
    //     }
    //     return false;
    // }

    return oneSequentialCommand.isFinished() || !oneSequentialCommand.isScheduled();

    // failsafe end the command
    // Logger.getGlobal().log(Level.WARNING, "Non stage failsafe allow");
    // return true;
  }

  /**
   * Creates and follows a path based on what we can see with vision goals: Auto can do 2 things 1)
   * drop off preloaded coral and do cycles between coral station and reef 2) drop off preloaded
   * coral and do cycles between reef(algae) and processor
   *
   * @param CoralStationChoose The current prefered Coral Station (We will cycle to and from this)
   */
  public void CreatePath(String CoralStationChoose) {
    //   switch (stage) {
    //       // move to reef (pathplanner)
    //     case 0:
    //       Logger.getGlobal().log(Level.INFO, "Started Stage: 0");
    //       moveCommand0.schedule();
    //       break;
    //     case 1:
    //       Logger.getGlobal().log(Level.INFO, "Started Stage: 1");
    //       sequentialMoveCommand1.schedule();
    //       break;
    //     case 2:
    //       Logger.getGlobal().log(Level.INFO, "Started Stage: 2");
    //       sequentialMoveCommand2.schedule();
    //       break;
    //     case 3:
    //       Logger.getGlobal().log(Level.INFO, "Started Stage: 3");
    //       sequentialMoveCommand3.schedule();
    //       break;
    //     case 4:
    //       Logger.getGlobal().log(Level.INFO, "Started Stage: 4");
    //       moveCommand4 = new AlgaeIntakeCommand(RobotContainer.algaeSubsystem);
    //       moveCommand4.schedule();
    //       break;
    //     case 5:
    //       Logger.getGlobal().log(Level.INFO, "Started Stage: 5");
    //       sequentialMoveCommand5.schedule();
    //       break;
    //     case 6:
    //       Logger.getGlobal().log(Level.INFO, "Started Stage: 6");
    //       moveCommand6 = RobotContainer.getCoralPathCommand(CoralStationChoose);
    //       moveCommand6.schedule();
    //       break;
    //     case 7:
    //       Logger.getGlobal().log(Level.INFO, "Started Stage: 7");
    //       sequentialMoveCommand7.schedule();
    //       break;
    //     case 8:
    //       Logger.getGlobal().log(Level.INFO, "Started Stage: 8");
    //       moveCommand8 = new GoToCommand(6);
    //       moveCommand8.schedule();
    //       break;
    //     case 9:
    //       Logger.getGlobal().log(Level.INFO, "Started Stage: 9");
    //       sequentialMoveCommand9.schedule();
    //       break;
    //     case 10:
    //       Logger.getGlobal().log(Level.INFO, "Started Stage: 10");
    //       sequentialMoveCommand10.schedule();
    //       break;
    //   }
    oneSequentialCommand.schedule();
  }
}
