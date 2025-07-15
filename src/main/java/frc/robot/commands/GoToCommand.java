package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.VisionSubsystem;

public class GoToCommand extends Command {

  public static Pose2d TagPos;
  int tagNum;

  public GoToCommand(int tag) {
    TagPos = AlignWithNearest.TagPos[tag-1];
    tagNum = tag;
  }

  @Override
  public void initialize() {
    Command command = RobotContainer.drivebase.driveToPose(TagPos);
    ParallelRaceGroup parallelRaceGroup =
        // Run until we can see the tag or the driver presses the back button
        command.until(
            () ->
                (VisionSubsystem.CanSeeTag(tagNum)
                    || RobotContainer.driverXbox.back().getAsBoolean()));
    parallelRaceGroup.schedule();
  }

  @Override
  public boolean isFinished() {
    return VisionSubsystem.CanSeeTag(tagNum);
  }
}
