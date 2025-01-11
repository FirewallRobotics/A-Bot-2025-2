package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.RawFiducial;
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
}
