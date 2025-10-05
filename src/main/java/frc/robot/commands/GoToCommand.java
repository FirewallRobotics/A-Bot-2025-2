package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.RobotContainer;

public class GoToCommand extends Command {

  public static Pose2d TagPos;
  int tagNum;
  ParallelRaceGroup parallelRaceGroup;

  public GoToCommand(int tag) {
    TagPos =
        new Pose2d(
            AlignWithNearest.TagPos[tag - 1][0],
            AlignWithNearest.TagPos[tag - 1][1],
            new Rotation2d(AlignWithNearest.TagPos[tag - 1][2]));
    tagNum = tag;
  }

  @Override
  public void initialize() {
    Command command = RobotContainer.drivebase.driveToPose(TagPos, 2, 2);
    parallelRaceGroup =
        // Run until we can see the tag or the driver presses the back button
        command.until(
            () ->
                (RobotContainer.driverXbox.back().getAsBoolean()
                    || RobotContainer.coralController.back().getAsBoolean()));
    parallelRaceGroup.schedule();
  }

  @Override
  public boolean isFinished() {
    return parallelRaceGroup.isFinished();
  }
}
