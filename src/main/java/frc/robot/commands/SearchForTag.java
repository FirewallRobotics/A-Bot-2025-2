package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem; // Update import path
import java.util.logging.Logger;

/*  Create SearchForTag.java class that extends Command.

The execute function checks for AprilTags in the current Limelight camera view by searching for the latest entries in the NetworkTables. If there aren't any current entries under 30 microseconds, the TimedRobot should start spinning in a clockwise direction using the swerve drive commands. It will stop when an AprilTag has been posted to the NetworkTables by the LimeLight system.

The class should be placed with the other commands under {project-root}src/main/java/frc/robot/commands.

Utilize create appropriate logging message following the best practices in {project-root}/doc/LOGGING.md
 */

// This class will be implemented to search for AprilTags using the Limelight camera.
// It will check for entries in NetworkTables and control the robot's movement accordingly.

// The execute function will contain logic to search for tags and control the robot's movement.
public class SearchForTag extends Command {
  private static final Logger logger = Logger.getLogger(SearchForTag.class.getName());
  private final SwerveSubsystem swerveDrive; // Change type to SwerveSubsystem
  private final Timer timer = new Timer();
  private static final double TIME_THRESHOLD = 0.03; // 30 milliseconds

  public SearchForTag(SwerveSubsystem swerveDrive) { // Change parameter type to SwerveSubsystem
    this.swerveDrive = swerveDrive;
    addRequirements(swerveDrive);
  }

  @Override
  public void initialize() {
    logger.info("SearchForTag command initialized.");
    timer.reset();
    timer.start();
  }

  @Override
  public void execute() {
    var table = NetworkTableInstance.getDefault().getTable("limelight");
    var tagEntries = table.getEntry("aprilTags");
    var lastUpdateTime = tagEntries.getLastChange() / 1e6; // Convert to seconds

    if (Timer.getFPGATimestamp() - lastUpdateTime > TIME_THRESHOLD) {
      logger.info("No recent AprilTag detected. Spinning robot clockwise.");
      // Create rotation using ChassisSpeeds
      var rotation = new edu.wpi.first.math.kinematics.ChassisSpeeds(0, 0, 0.3);
      swerveDrive.drive(rotation);
    } else {
      logger.info("AprilTag detected. Stopping robot.");
      // Stop by sending zero speeds
      swerveDrive.drive(new edu.wpi.first.math.kinematics.ChassisSpeeds());
    }
  }

  @Override
  public boolean isFinished() {
    return false; // This command runs until explicitly canceled.
  }

  @Override
  public void end(boolean interrupted) {
    logger.info("SearchForTag command ended. Stopping robot.");
    // Stop by sending zero speeds
    swerveDrive.drive(new edu.wpi.first.math.kinematics.ChassisSpeeds());
    timer.stop();
  }
}
