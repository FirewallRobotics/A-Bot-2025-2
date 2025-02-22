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
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.LimelightHelpers.LimelightTarget_Retro;
import frc.robot.RobotContainer;
import java.util.List;
import java.util.Optional;
import java.util.function.DoubleSupplier;

public class FlexAutoSubsystem implements Pathfinder {

  List<Waypoint> waypoints;
  List<Pose2d> poseway;
  double[] RobotSpaceCoralLocation;
  double[] RobotSpaceAlgaeLocation;
  double[] ReefLocation;
  double[] ProcLocation;
  Pose2d RobotFieldSpace;

  // create a selector for flex auto modes
  private final SendableChooser<String> m_AutoObjChooser = new SendableChooser<>();

  public FlexAutoSubsystem() {
    // add selections for flex auto to smartdashboard
    m_AutoObjChooser.setDefaultOption("Coral", "coral");
    m_AutoObjChooser.addOption("Algae", "algae");
    SmartDashboard.putData("Auto Obj choices", m_AutoObjChooser);
    SmartDashboard.putNumber("AutoScanSpeed", 1.0);
    SmartDashboard.putNumber("Ultrasonics Coral", 50);
    SmartDashboard.putNumber("Ultrasonics Algae", 50);
    SmartDashboard.putNumber("AutoMoveSpeed", 1.0);
    SmartDashboard.putNumber("AutoRotateSpeed", 1.0);
  }

  /**
   * Get if a new path has been calculated since the last time a path was retrieved
   *
   * @return True if a new path is available
   */
  @Override
  public boolean isNewPathAvailable() {
    // new path is avaliable if we are not moving
    if (RobotContainer.drivebase.getRobotVelocity().vxMetersPerSecond < 1
        || RobotContainer.drivebase.getRobotVelocity().vyMetersPerSecond < 1) {
      return true;
    }
    return false;
  }

  public Translation2d getReefLocationInFieldSpace() {

    // get reef location in robot space
    ReefLocation = VisionSubsystem.getReefLocation();

    // if we dont have the reefs location find it by spinning slowly
    if (ReefLocation[0] == -1 && ReefLocation[1] == -1) {
      DoubleSupplier scanspeed = () -> SmartDashboard.getNumber("AutoScanSpeed", 1.0);
      RobotContainer.drivebase.driveCommand(() -> 0, () -> 0, scanspeed);
    } else {
      // if we do have the reefs location then convert it
      // first zero the drivecommand so the math stays right
      RobotContainer.drivebase.driveCommand(() -> 0, () -> 0, () -> 0);

      // get the robots location in field space
      RobotFieldSpace = LimelightTarget_Retro.getRobotPose_FieldSpace2D();

      // do the math to find the location of the reef by adding together the values
      double xActual = ReefLocation[0] + RobotFieldSpace.getX();
      double yActual = ReefLocation[1] + RobotFieldSpace.getY();

      // return the values
      return new Translation2d(xActual, yActual);
    }
    return null;
  }

  public Translation2d getProcessorLocationInFieldSpace() {

    // get Processor location in robot space
    ProcLocation = VisionSubsystem.getProcessorLocation();

    // if we dont have its location find it by spinning slowly
    if (ProcLocation[0] == -1 && ProcLocation[1] == -1) {
      DoubleSupplier scanspeed = () -> SmartDashboard.getNumber("AutoScanSpeed", 1.0);
      RobotContainer.drivebase.driveCommand(null, null, scanspeed);
    } else {
      // if we do have its location then convert it
      // first zero the drivecommand so the math stays right
      RobotContainer.drivebase.driveCommand(null, null, null);

      // get the robots location in field space
      RobotFieldSpace = LimelightTarget_Retro.getRobotPose_FieldSpace2D();

      // do the math to find the location of it by adding together the values
      double xActual = ProcLocation[0] + RobotFieldSpace.getX();
      double yActual = ProcLocation[1] + RobotFieldSpace.getY();

      // return the values
      return new Translation2d(xActual, yActual);
    }
    return null;
  }

  /**
   * Use the inputed goalState to make a new path
   *
   * @param constraints The path constraints to use when creating the path
   * @param goalEndState The goal end state to use when creating the path
   * @return The PathPlannerPath created from the points calculated by the pathfinder
   */
  @Override
  public PathPlannerPath getCurrentPath(PathConstraints constraints, GoalEndState goalEndState) {

    // goals: Auto can do 2 things
    // 1) drop off preloaded coral and do cycles between coral station and reef
    // 2) drop off preloaded coral and do cycles between reef(algae) and processor

    // if our plan is to go to the coral station then were doing coral station cycles
    if (SmartDashboard.getBoolean("AutoThenGoToCoralStation", false)) {

      // if a new path is available AKA we are not moving
      if (isNewPathAvailable()) {

        // get the robots location in field space
        Pose3d temp = VisionSubsystem.getRobotPoseInFieldSpace();

        // set the start position to our current location
        setStartPosition(new Translation2d(temp.getX(), temp.getY()));

        // get our Alliance
        Optional<Alliance> ally = DriverStation.getAlliance();
        if (ally.isPresent()) {
          if (ally.get() == Alliance.Blue) {
            // if we are on blue then go to the blue coral station and reef
            setGoalPosition(new Translation2d(4, 4));
            waypoints.add(
                new Waypoint(
                    new Translation2d(temp.getX(), temp.getY()),
                    getReefLocationInFieldSpace(),
                    new Translation2d(4, 4)));
          }
          if (ally.get() == Alliance.Red) {
            // if we are red go to the red coral station and reef
            setGoalPosition(new Translation2d(13, 4));
            waypoints.add(
                new Waypoint(
                    new Translation2d(temp.getX(), temp.getY()),
                    getReefLocationInFieldSpace(),
                    new Translation2d(13, 4)));
          }
        }
      }
      // if we are not going to go to the coral station then we are doing algae cycles
    } else {

      // if a new path is available AKA we are not moving
      if (isNewPathAvailable()) {

        // get our current location in field space
        Pose3d temp = VisionSubsystem.getRobotPoseInFieldSpace();

        // set the start position to our location
        setStartPosition(new Translation2d(temp.getX(), temp.getY()));

        // set our goal to the processor
        setGoalPosition(getProcessorLocationInFieldSpace());

        // go to the reef
        waypoints.add(
            new Waypoint(
                new Translation2d(temp.getX(), temp.getY()),
                getReefLocationInFieldSpace(),
                getProcessorLocationInFieldSpace()));
      }
    }

    // wrap this up into a path we can follow
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
    // Create and push Field2d to SmartDashboard.

    @SuppressWarnings("resource")
    // complains as it thinks that field will not be closed but it in fact IS
    Field2d m_field = new Field2d();

    m_field = (Field2d) SmartDashboard.getData("Field");

    // Push the trajectory to Field2d.
    if (poseway != null) {
      Trajectory m_trajectory =
          TrajectoryGenerator.generateTrajectory(
              poseway, new TrajectoryConfig(Units.feetToMeters(3.0), Units.feetToMeters(3.0)));
      m_field.getObject("traj").setTrajectory(m_trajectory);
    }
    m_field.close();
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
    // unused
  }

  /**
   * Set the goal position to pathfind to
   *
   * @param goalPosition Goal position on the field. f this is within an obstacle it will be moved
   *     to the nearest non-obstacle node.
   */
  @Override
  public void setGoalPosition(Translation2d goalPosition) {
    // unused
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
