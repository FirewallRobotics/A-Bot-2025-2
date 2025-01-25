package frc.robot.subsystems;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.pathfinding.Pathfinder;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.LimelightHelpers.LimelightTarget_Retro;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import java.util.List;
import java.util.function.DoubleSupplier;

public class FlexAutoSubsystem implements Pathfinder {

  List<Waypoint> waypoints;
  List<Translation2d> GoToPoints;
  double[] RobotSpaceCoralLocation;
  double[] RobotSpaceAlgaeLocation;
  double[] ReefLocation;
  double[] ProcLocation;
  Pose2d RobotFieldSpace;

  private final SendableChooser<String> m_AutoObjChooser = new SendableChooser<>();

  public FlexAutoSubsystem() {
    m_AutoObjChooser.setDefaultOption("Coral", "coral");
    m_AutoObjChooser.addOption("Algae", "algae");
    SmartDashboard.putData("Auto  Obj choices", m_AutoObjChooser);
  }

  private final SwerveSubsystem drivebase =
      new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve/neo"));

  /**
   * Get if a new path has been calculated since the last time a path was retrieved
   *
   * @return True if a new path is available
   */
  @Override
  public boolean isNewPathAvailable() {
    // new path is avaliable if:
    // coral is in view and we dont have a coral
    // we just picked up a coral
    // we got to the end of the current path
    if (VisionSubsystem.getCoralLocationCamera() != null
        || drivebase.getRobotVelocity().vxMetersPerSecond < 1
        || drivebase.getRobotVelocity().vyMetersPerSecond
            < 1) { // TODO: Add coral sensor to this statement
      return true;
    }
    return false;
  }

  public Translation2d getReefLocationInFieldSpace() {
    ReefLocation = VisionSubsystem.getReefLocation();
    if (ReefLocation[0] == -1 && ReefLocation[1] == -1) {
      DoubleSupplier scanspeed = () -> SmartDashboard.getNumber("AutoScanSpeed", 1.0);
      drivebase.driveCommand(null, null, scanspeed);
    } else {
      drivebase.driveCommand(null, null, null);
      RobotFieldSpace = LimelightTarget_Retro.getRobotPose_FieldSpace2D();
      double xActual = ReefLocation[0] + RobotFieldSpace.getX();
      double yActual = ReefLocation[1] + RobotFieldSpace.getY();
      return new Translation2d(xActual, yActual);
    }
    return null;
  }

  public Translation2d getProcessorLocationInFieldSpace() {
    ProcLocation = VisionSubsystem.getProcessorLocation();
    if (ProcLocation[0] == -1 && ProcLocation[1] == -1) {
      DoubleSupplier scanspeed = () -> SmartDashboard.getNumber("AutoScanSpeed", 1.0);
      drivebase.driveCommand(null, null, scanspeed);
    } else {
      drivebase.driveCommand(null, null, null);
      RobotFieldSpace = LimelightTarget_Retro.getRobotPose_FieldSpace2D();
      double xActual = ProcLocation[0] + RobotFieldSpace.getX();
      double yActual = ProcLocation[1] + RobotFieldSpace.getY();
      return new Translation2d(xActual, yActual);
    }
    return null;
  }

  /**
   * Get the most recently calculated path
   *
   * @param constraints The path constraints to use when creating the path
   * @param goalEndState The goal end state to use when creating the path
   * @return The PathPlannerPath created from the points calculated by the pathfinder
   */
  @Override
  public PathPlannerPath getCurrentPath(PathConstraints constraints, GoalEndState goalEndState) {
    if (m_AutoObjChooser.getSelected().equals("coral")) {
      if (isNewPathAvailable()) { // TODO: and we have a coral (use coral sensor)
        Pose3d temp = VisionSubsystem.getRobotPoseInFieldSpace();
        while (GoToPoints.size() != 0) {
          GoToPoints.remove(0);
        }
        setStartPosition(new Translation2d(temp.getX(), temp.getY()));
        setGoalPosition(getReefLocationInFieldSpace());
      }
      // TODO: when we don't have a coral (use coral sensor)
      if (isNewPathAvailable() && false) {
        while (VisionSubsystem.getCoralLocationCamera()[0] > 0) {
          DoubleSupplier rotspeed = () -> SmartDashboard.getNumber("AutoRotateSpeed", 1.0);
          drivebase.driveCommand(null, null, rotspeed);
        }
        while (VisionSubsystem.getCoralLocationCamera()[0] < 0) {
          DoubleSupplier rotspeed = () -> -SmartDashboard.getNumber("AutoRotateSpeed", 1.0);
          drivebase.driveCommand(null, null, rotspeed);
        }
        if (VisionSubsystem.getCoralLocationCamera()[0] == 0
            && SmartDashboard.getBoolean("AutoCanMove", false)) {
          while (VisionSubsystem.getCoralLocationCamera() != null) {
            DoubleSupplier movespeed = () -> SmartDashboard.getNumber("AutoMoveSpeed", 1.0);
            drivebase.driveCommand(movespeed, null, null);
          }
        }
      }
    } else {
      if (isNewPathAvailable()) { // TODO: and we have an algae (use algae sensor)
        Pose3d temp = VisionSubsystem.getRobotPoseInFieldSpace();
        while (GoToPoints.size() != 0) {
          GoToPoints.remove(0);
        }
        setStartPosition(new Translation2d(temp.getX(), temp.getY()));
        setGoalPosition(getProcessorLocationInFieldSpace());
      }
      // TODO: when we don't have a coral (use coral sensor)
      if (isNewPathAvailable() && false) {
        while (VisionSubsystem.getCoralLocationCamera()[0] > 0) {
          DoubleSupplier rotspeed = () -> SmartDashboard.getNumber("AutoRotateSpeed", 1.0);
          drivebase.driveCommand(null, null, rotspeed);
        }
        while (VisionSubsystem.getCoralLocationCamera()[0] < 0) {
          DoubleSupplier rotspeed = () -> -SmartDashboard.getNumber("AutoRotateSpeed", 1.0);
          drivebase.driveCommand(null, null, rotspeed);
        }
        if (VisionSubsystem.getCoralLocationCamera()[0] == 0
            && SmartDashboard.getBoolean("AutoCanMove", false)) {
          while (VisionSubsystem.getCoralLocationCamera() != null) {
            DoubleSupplier movespeed = () -> SmartDashboard.getNumber("AutoMoveSpeed", 1.0);
            drivebase.driveCommand(movespeed, null, null);
          }
        }
      }
    }
    for (int i = 0; i < GoToPoints.size(); i++) {
      if (i == 0) {
        waypoints.add(new Waypoint(GoToPoints.get(i), GoToPoints.get(i), GoToPoints.get(i + 1)));
      } else if (i + 1 == GoToPoints.size()) {
        waypoints.add(new Waypoint(GoToPoints.get(i - 1), GoToPoints.get(i), GoToPoints.get(i)));
      } else {
        waypoints.add(
            new Waypoint(GoToPoints.get(i - 1), GoToPoints.get(i), GoToPoints.get(i + 1)));
      }
    }
    PathPlannerPath path =
        new PathPlannerPath(
            waypoints,
            constraints,
            null, // The ideal starting state, this is only relevant for pre-planned paths, so can
            // be null for on-the-fly paths.
            new GoalEndState(
                0.0,
                Rotation2d.fromDegrees(
                    -90)) // Goal end state. You can set a holonomic rotation here. If using a
            // differential drivetrain, the rotation will have no effect.
            );
    return path;
  }

  /**
   * Set the start position to pathfind from
   *
   * @param startPosition Start position on the field. If this is within an obstacle it will be
   *     moved to the nearest non-obstacle node.
   */
  @Override
  public void setStartPosition(Translation2d startPosition) {
    if (GoToPoints.size() == 0) {
      GoToPoints.add(startPosition);
    } else {
      GoToPoints.set(0, startPosition);
    }
  }

  /**
   * Set the goal position to pathfind to
   *
   * @param goalPosition Goal position on the field. f this is within an obstacle it will be moved
   *     to the nearest non-obstacle node.
   */
  @Override
  public void setGoalPosition(Translation2d goalPosition) {
    if (GoToPoints.size() == 0) {
      GoToPoints.add(goalPosition);
    } else {
      GoToPoints.set(GoToPoints.size() - 1, goalPosition);
    }
  }

  /**
   * Set the dynamic obstacles that should be avoided while pathfinding.
   *
   * @param obs A List of Translation2d pairs representing obstacles. Each Translation2d represents
   *     opposite corners of a bounding box.
   * @param currentRobotPos The current position of the robot. This is needed to change the start
   *     position of the path to properly avoid obstacles
   */
  @Override
  public void setDynamicObstacles(
      List<Pair<Translation2d, Translation2d>> obs, Translation2d currentRobotPos) {
    // non used
  }
}
