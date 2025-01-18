package frc.robot.subsystems;

import java.io.File;
import java.util.List;
import java.util.function.DoubleSupplier;

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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.LimelightHelpers.LimelightTarget_Retro;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class FlexAutoSubsystem implements Pathfinder {


  List<Waypoint> waypoints;
  List<Translation2d> GoToPoints;
  double[] RobotSpaceCoralLocation;
  double[] RobotSpaceAlgaeLocation;
  double[] ReefLocation;
  Pose2d RobotFieldSpace;

  private final SwerveSubsystem       drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                "swerve/neo"));

    /**
   * Get if a new path has been calculated since the last time a path was retrieved
   *
   * @return True if a new path is available
   */
  @Override
  public boolean isNewPathAvailable() {
    for(int i = 0; i < GoToPoints.size(); i++){
      Pose3d temp = VisionSubsystem.getRobotPoseInFieldSpace();
      if(temp.getX() == GoToPoints.get(GoToPoints.size()-1).getX()){
        if(temp.getY() == GoToPoints.get(GoToPoints.size()-1).getY()){
          if(!GoToPoints.get(i).equals(GetAlgaeLocationInFieldSpace()) || !GoToPoints.get(i).equals(getReefLocationInFieldSpace())){
              return true;
          }
        }
      }
    }
    return false;
  }

  public Translation2d GetCoralLocationInFieldSpace(){
    RobotSpaceCoralLocation = VisionSubsystem.getCoralLocation();
    RobotFieldSpace = LimelightTarget_Retro.getRobotPose_FieldSpace2D();
    double xActual = RobotSpaceCoralLocation[0] + RobotFieldSpace.getX();
    double yActual = RobotSpaceCoralLocation[1] + RobotFieldSpace.getY();
    return new Translation2d(xActual,yActual);
  }

  public Translation2d GetAlgaeLocationInFieldSpace(){
    RobotSpaceAlgaeLocation = VisionSubsystem.getAlgaeLocation();
    RobotFieldSpace = LimelightTarget_Retro.getRobotPose_FieldSpace2D();
    double xActual = RobotSpaceAlgaeLocation[0] + RobotFieldSpace.getX();
    double yActual = RobotSpaceAlgaeLocation[1] + RobotFieldSpace.getY();
    return new Translation2d(xActual,yActual);
  }

  public Translation2d getReefLocationInFieldSpace(){
    ReefLocation = VisionSubsystem.getReefLocation();
    if(ReefLocation[0] == -1 && ReefLocation[1] == -1){
        DoubleSupplier scanspeed = () -> SmartDashboard.getNumber("AutoScanSpeed", 1.0);
        drivebase.driveCommand(null, null, scanspeed);
    }
    RobotFieldSpace = LimelightTarget_Retro.getRobotPose_FieldSpace2D();
    double xActual = ReefLocation[0] + RobotFieldSpace.getX();
    double yActual = ReefLocation[1] + RobotFieldSpace.getY();
    return new Translation2d(xActual,yActual);
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
    if(isNewPathAvailable()){
      Pose3d temp = VisionSubsystem.getRobotPoseInFieldSpace();
      while(GoToPoints.size() != 0){
        GoToPoints.remove(0);
      }
      setStartPosition(new Translation2d(temp.getX(), temp.getY()));
      GoToPoints.add(GetAlgaeLocationInFieldSpace());
      setGoalPosition(getReefLocationInFieldSpace());
    }
    for(int i = 0; i < GoToPoints.size(); i++){
        if(i == 0){
            waypoints.add(new Waypoint(GoToPoints.get(i),GoToPoints.get(i),GoToPoints.get(i+1)));
        }
        else if(i+1 == GoToPoints.size()){
            waypoints.add(new Waypoint(GoToPoints.get(i-1),GoToPoints.get(i),GoToPoints.get(i)));
        }else{
            waypoints.add(new Waypoint(GoToPoints.get(i-1),GoToPoints.get(i),GoToPoints.get(i+1)));
        }
    }
    PathPlannerPath path = new PathPlannerPath(
            waypoints,
            constraints,
            null, // The ideal starting state, this is only relevant for pre-planned paths, so can be null for on-the-fly paths.
            new GoalEndState(0.0, Rotation2d.fromDegrees(-90)) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
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
    if(GoToPoints.size() == 0){
        GoToPoints.add(startPosition);
    }
    else{
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
    if(GoToPoints.size() == 0){
        GoToPoints.add(goalPosition);
    }
    else{
        GoToPoints.set(GoToPoints.size()-1, goalPosition);
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
