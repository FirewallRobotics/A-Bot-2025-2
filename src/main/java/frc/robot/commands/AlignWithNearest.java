package frc.robot.commands;

import java.io.File;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class AlignWithNearest extends Command {

    //setup swerve
    private final SwerveSubsystem drivebase =
        new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve/neo"));

    //add vision as a requirement to run
    public AlignWithNearest(VisionSubsystem visionSubsystem){
        addRequirements(visionSubsystem);
    }

    @Override
    public void initialize() {

        //if we are close enough
        if(VisionSubsystem.DistanceToReef() < SmartDashboard.getNumber("AssistMinDistance", 10)){
            LimelightHelpers.setPipelineIndex("",2);
            while(LimelightHelpers.getTX("") > 1){
                DoubleSupplier movespeed = () -> SmartDashboard.getNumber("AutoMoveSpeed", 1.0);
                drivebase.driveCommand(movespeed, null, null);
            }
            while(LimelightHelpers.getTX("") < -1){
                DoubleSupplier movespeed = () -> -SmartDashboard.getNumber("AutoMoveSpeed", 1.0);
                drivebase.driveCommand(movespeed, null, null);
            }
        }
    }

    @Override
    public boolean isFinished() {
        return LimelightHelpers.getTX("") < 1;
    }
}
