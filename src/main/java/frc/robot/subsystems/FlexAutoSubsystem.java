package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
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

    oneSequentialCommand =
        new SequentialCommandGroup(
            RobotContainer.drivebase.driveToPose(AlignWithNearest.TagPos[6 - 1], 2, 2),
            new ElevatorMoveLevel2(RobotContainer.elevatorCoralSubsystem),
            new ArmSetToMiddle(RobotContainer.coralWristSubsystem),
            new AlignWithNearest(0, null, null),
            // new ArmSetToScore(RobotContainer.coralWristSubsystem),
            new ParallelRaceGroup(
                new CoralShootCommand(RobotContainer.coralHoldSubsystem), new WaitCommand(0.25)),
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

    return oneSequentialCommand.isFinished() || !oneSequentialCommand.isScheduled();
  }

  /**
   * Creates and follows a path based on what we can see with vision goals: Auto can do 2 things 1)
   * drop off preloaded coral and do cycles between coral station and reef 2) drop off preloaded
   * coral and do cycles between reef(algae) and processor
   *
   * @param CoralStationChoose The current prefered Coral Station (We will cycle to and from this)
   */
  public void CreatePath(String CoralStationChoose) {

    oneSequentialCommand.schedule();
  }
}
