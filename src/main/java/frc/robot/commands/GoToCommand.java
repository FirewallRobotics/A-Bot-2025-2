package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.VisionSubsystem;
import java.util.logging.Level;
import java.util.logging.Logger;

public class GoToCommand extends Command {

  public static Pose2d TagPos;
  int tagNum;

  public GoToCommand(int tag) {
    TagPos = AlignWithNearest.TagPos[tag];
    tagNum = tag;
  }

  @Override
  public void initialize() {
    Logger.getGlobal().log(Level.INFO, "Going to tag: " + tagNum);
    RobotContainer.drivebase.driveToPose(TagPos);
  }

  @Override
  public boolean isFinished() {
    return VisionSubsystem.CanSeeTag(tagNum);
  }
}
