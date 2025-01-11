package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.RawFiducial;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class VisionSubsystem extends SubsystemBase {

    //pipeline layout:
    //0 - april tags
    //1 - Coral / white mask
    //2 - Algae / Circlular Green mask

    private int[] reefTags = {6,7,8,9,10,11,17,18,19,20,21,22};
    private int[] coralTags = {1,2,12,13};
    private int[] processorTags = {3,16};
    private int[] bargeTags = {4,5,14,15};

    @Override
    public void periodic(){
        //switch pipeline to april tags
        LimelightHelpers.setPipelineIndex("", 0);

        LimelightHelpers.SetFiducialIDFiltersOverride("", new int[]{
            0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22});
        LimelightHelpers.SetFiducialDownscalingOverride("", 1.5f);

        // send targets to drivers
        SmartDashboard.putNumber("AprilTags Found:", LimelightHelpers.getTargetCount(getName()));

        // First, tell Limelight your robot's current orientation
        //double robotYaw = m_gyro.getYaw(); todo: when movement code works add gyro
        //LimelightHelpers.SetRobotOrientation("", robotYaw, 0.0, 0.0, 0.0, 0.0, 0.0);
    }

    public double DistanceToReef(){
        RawFiducial[] fiducials = LimelightHelpers.getRawFiducials("");
        double shortest = Double.MAX_VALUE;
        for (RawFiducial fiducial : fiducials) {
            int id = fiducial.id;                    // Tag ID
            double distToRobot = fiducial.distToRobot;    // Distance to robot
            for(int i = 0; i < reefTags.length; i++){
                if(id == reefTags[i]){
                    shortest = distToRobot;
                }
            }
        }
        if(shortest != Double.MAX_VALUE){
            return shortest;
        }
        else{
            return -1;
        }
    }

    public double DistanceToCoralStation(){
        RawFiducial[] fiducials = LimelightHelpers.getRawFiducials("");
        double shortest = Double.MAX_VALUE;
        for (RawFiducial fiducial : fiducials) {
            int id = fiducial.id;                    // Tag ID
            double distToRobot = fiducial.distToRobot;    // Distance to robot
            for(int i = 0; i < coralTags.length; i++){
                if(id == coralTags[i]){
                    shortest = distToRobot;
                }
            }
        }
        if(shortest != Double.MAX_VALUE){
            return shortest;
        }
        else{
            return -1;
        }
    }

    public double DistanceToProcessor(){
        RawFiducial[] fiducials = LimelightHelpers.getRawFiducials("");
        double shortest = Double.MAX_VALUE;
        for (RawFiducial fiducial : fiducials) {
            int id = fiducial.id;                    // Tag ID
            double distToRobot = fiducial.distToRobot;    // Distance to robot
            for(int i = 0; i < processorTags.length; i++){
                if(id == processorTags[i]){
                    shortest = distToRobot;
                }
            }
        }
        if(shortest != Double.MAX_VALUE){
            return shortest;
        }
        else{
            return -1;
        }
    }

    public double DistanceToBarge(){
        RawFiducial[] fiducials = LimelightHelpers.getRawFiducials("");
        double shortest = Double.MAX_VALUE;
        for (RawFiducial fiducial : fiducials) {
            int id = fiducial.id;                    // Tag ID
            double distToRobot = fiducial.distToRobot;    // Distance to robot
            for(int i = 0; i < bargeTags.length; i++){
                if(id == bargeTags[i]){
                    shortest = distToRobot;
                }
            }
        }
        if(shortest != Double.MAX_VALUE){
            return shortest;
        }
        else{
            return -1;
        }
    }

    public static Command FlexAuto(){
        // Create a list of waypoints from poses. Each pose represents one waypoint.
        // The rotation component of the pose should be the direction of travel. Do not use holonomic rotation.
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
            new Pose2d(1.0, 1.0, Rotation2d.fromDegrees(0)),
            new Pose2d(3.0, 1.0, Rotation2d.fromDegrees(0)),
            new Pose2d(5.0, 3.0, Rotation2d.fromDegrees(90))
        );

        PathConstraints constraints = new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI); // The constraints for this path.
        // PathConstraints constraints = PathConstraints.unlimitedConstraints(12.0); // You can also use unlimited constraints, only limited by motor torque and nominal battery voltage

        // Create the path using the waypoints created above
        PathPlannerPath path = new PathPlannerPath(
            waypoints,
            constraints,
            null, // The ideal starting state, this is only relevant for pre-planned paths, so can be null for on-the-fly paths.
            new GoalEndState(0.0, Rotation2d.fromDegrees(-90)) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
        );
        return AutoBuilder.followPath(path);

    }
}
