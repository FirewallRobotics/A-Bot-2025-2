package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.AllianceFlipUtil;
import java.util.logging.Level;
import java.util.logging.Logger;

public class AlignWithNearest extends Command {

  public Pose2d getSelectedPose() {
    Logger.getGlobal()
        .log(
            Level.INFO,
            "Getting pose for tag: " + Robot.desiredScoreSendableChooser.getSelected().toString());
    return AllianceFlipUtil.apply(Robot.desiredScoreSendableChooser.getSelected().scorePosition);
  }

  public static String name = frc.robot.Constants.VisionSubsystemConstants.limelightName;
  private Command pathCommand;

  private Pose2d targetPose;
  private double distanceAway = -0.55;

  public static Pose2d[] TagPos = {
    new Pose2d(16.408, 1.048, new Rotation2d(-0.9075712)),
    new Pose2d(16.296, 7.007, new Rotation2d(0.9075712)),
    new Pose2d(11.434, 7.398, new Rotation2d(1.570796)),
    new Pose2d(0, 0, new Rotation2d(0)),
    new Pose2d(0, 0, new Rotation2d(0)),
    new Pose2d(13.787, 2.811, new Rotation2d(2.094395)),
    new Pose2d(14.261, 2.220, new Rotation2d(Math.toRadians(125))),
    new Pose2d(13.840, 5.217, new Rotation2d(-2.111848)),
    new Pose2d(12.365, 5.165, new Rotation2d(-1.012291)),
    new Pose2d(11.638, 4.007, new Rotation2d(0)),
    new Pose2d(12.390, 2.790, new Rotation2d(1.012291)),
    new Pose2d(1.161, 1.048, new Rotation2d(-2.216568)),
    new Pose2d(1.131, 6.950, new Rotation2d(2.181662)),
    new Pose2d(0, 0, new Rotation2d(0)),
    new Pose2d(0, 0, new Rotation2d(0)),
    new Pose2d(6.364, 0.550, new Rotation2d(-1.570796)),
    new Pose2d(3.390, 2.790, new Rotation2d(1.012291)),
    new Pose2d(2.638, 4.007, new Rotation2d(0)),
    new Pose2d(3.365, 5.165, new Rotation2d(-1.012291)),
    new Pose2d(4.840, 5.217, new Rotation2d(-2.111848)),
    new Pose2d(5.538, 3.969, new Rotation2d(3.141593)),
    new Pose2d(4.787, 2.811, new Rotation2d(2.094395))
  };

  public static Pose2d Tag13 = new Pose2d(1.131, 6.950, new Rotation2d(2.181662));
  public static Pose2d Tag12 = new Pose2d(1.161, 1.048, new Rotation2d(-2.216568));
  public static Pose2d Tag2 = new Pose2d(16.296, 7.007, new Rotation2d(0.9075712));
  public static Pose2d Tag1 = new Pose2d(16.408, 1.048, new Rotation2d(-0.9075712));

  public static Pose2d Tag3 = new Pose2d(11.434, 7.398, new Rotation2d(1.570796));
  public static Pose2d Tag16 = new Pose2d(6.364, 0.550, new Rotation2d(-1.570796));

  public static Pose2d Tag17 = new Pose2d(3.390, 2.790, new Rotation2d(1.012291));
  public static Pose2d Tag18 = new Pose2d(2.638, 4.007, new Rotation2d(0));
  public static Pose2d Tag19 = new Pose2d(3.365, 5.165, new Rotation2d(-1.012291));
  public static Pose2d Tag20 = new Pose2d(4.840, 5.217, new Rotation2d(-2.111848));
  public static Pose2d Tag21 = new Pose2d(5.538, 3.969, new Rotation2d(3.141593));
  public static Pose2d Tag22 = new Pose2d(4.787, 2.811, new Rotation2d(2.094395));

  public static Pose2d Tag6 = new Pose2d(13.787, 2.811, new Rotation2d(2.094395));
  public static Pose2d Tag7 = new Pose2d(14.538, 3.969, new Rotation2d(3.141593));
  public static Pose2d Tag8 = new Pose2d(13.840, 5.217, new Rotation2d(-2.111848));
  public static Pose2d Tag9 = new Pose2d(12.365, 5.165, new Rotation2d(-1.012291));
  public static Pose2d Tag10 = new Pose2d(11.638, 4.007, new Rotation2d(0));
  public static Pose2d Tag11 = new Pose2d(12.390, 2.790, new Rotation2d(1.012291));

  public Command targetCommand;

  // add vision as a requirement to run
  public AlignWithNearest() {}

  @Override
  public void initialize() {
    Pose2d selectedPosition = getSelectedPose();

    targetPose =
        new Pose2d(
            Math.cos(selectedPosition.getRotation().getRadians()) * distanceAway
                - Math.sin(selectedPosition.getRotation().getRadians())
                    * SmartDashboard.getNumber("getAutoAlignOffsetX", 0)
                + selectedPosition.getTranslation().getX(),
            Math.sin(selectedPosition.getRotation().getRadians()) * distanceAway
                + Math.cos(selectedPosition.getRotation().getRadians())
                    * SmartDashboard.getNumber("getAutoAlignOffsetX", 0)
                + selectedPosition.getTranslation().getY(),
            selectedPosition.getRotation());

    pathCommand = AutoBuilder.pathfindToPose(targetPose, new PathConstraints(1, 1, 180, 180));
  }

  public void execute() {
    // robotPose = VisionSubsystem.getRobotPoseInFieldSpace().toPose2d();

    // if (!robotPose.equals(new Pose2d())) RobotContainer.drivebase.driveToPose(robotPose);

    Logger.getGlobal().log(Level.WARNING, "Scheduling path");
    pathCommand.schedule();
  }

  @Override
  public void end(boolean inter) {
    pathCommand.end(inter);
  }

  @Override
  public boolean isFinished() {
    return pathCommand.isFinished();
  }
}
