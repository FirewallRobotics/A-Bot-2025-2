package frc.robot.commands;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import java.util.function.DoubleSupplier;

public class AlignWithNearest extends Command {

  // setup swerve
  private final SwerveSubsystem drivebase =
      new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve/neo"));

  // add vision as a requirement to run
  public AlignWithNearest(VisionSubsystem visionSubsystem) {
    addRequirements(visionSubsystem);
  }

  @Override
  public void initialize() {

    // drive commands take doubleSuppliers
    DoubleSupplier zero = () -> 0.0;

    // if we are close enough to the reef
    if (VisionSubsystem.DistanceToReef() < SmartDashboard.getNumber("AssistMinDistance", 10)) {
      // switch to the pipeline with the right crosshair position for the coral side of the elevator
      LimelightHelpers.setPipelineIndex("", 2);

      // if we are to much too the right move left
      while (LimelightHelpers.getTX("") > SmartDashboard.getNumber("AssistLossRange", 1.0)) {
        DoubleSupplier movespeed = () -> SmartDashboard.getNumber("AutoMoveSpeed", 1.0);
        drivebase.driveCommand(zero, movespeed, null);
      }

      // if we are too much to the left move right
      while (LimelightHelpers.getTX("") < -SmartDashboard.getNumber("AssistLossRange", 1.0)) {
        DoubleSupplier movespeed = () -> -SmartDashboard.getNumber("AutoMoveSpeed", 1.0);
        drivebase.driveCommand(zero, movespeed, null);
      }

      // if we are too far from the tag move forward
      while (VisionSubsystem.DistanceToReef()
          > SmartDashboard.getNumber("AssistReefDistance", 1.0)) {
        DoubleSupplier movespeed = () -> -SmartDashboard.getNumber("AutoMoveSpeed", 1.0);
        drivebase.driveCommand(movespeed, zero, null);
      }

      // we are now on target and can stop!
      drivebase.driveCommand(zero, zero, null);
    }

    // if we are close enough to the processor
    if (VisionSubsystem.DistanceToProcessor() < SmartDashboard.getNumber("AssistMinDistance", 10)) {
      // switch to the pipeline with the right crosshair position for the algae side of the elevator
      LimelightHelpers.setPipelineIndex("", 3);

      // if we are to much too the right move left
      while (LimelightHelpers.getTX("") > SmartDashboard.getNumber("AssistLossRange", 1.0)) {
        DoubleSupplier movespeed = () -> SmartDashboard.getNumber("AutoMoveSpeed", 1.0);
        drivebase.driveCommand(zero, movespeed, null);
      }

      // if we are too much to the left move right
      while (LimelightHelpers.getTX("") < -SmartDashboard.getNumber("AssistLossRange", 1.0)) {
        DoubleSupplier movespeed = () -> -SmartDashboard.getNumber("AutoMoveSpeed", 1.0);
        drivebase.driveCommand(zero, movespeed, null);
      }

      // if we are too far from the tag move forward
      while (VisionSubsystem.DistanceToProcessor()
          > SmartDashboard.getNumber("AssistProcessorDistance", 1.0)) {
        DoubleSupplier movespeed = () -> -SmartDashboard.getNumber("AutoMoveSpeed", 1.0);
        drivebase.driveCommand(movespeed, zero, null);
      }

      // we are now on target and can stop!
      drivebase.driveCommand(zero, zero, null);
    }
  }

  @Override
  public boolean isFinished() {

    // we are done if:
    // - we are within the range created by lossRange
    // - we are within the distance set by ReefDistance OR ProcessorDistance
    double temp = LimelightHelpers.getTX("");
    return temp < SmartDashboard.getNumber("AssistLossRange", 1.0)
        && temp > -SmartDashboard.getNumber("AssistLossRange", 1.0)
        && (VisionSubsystem.DistanceToReef() > SmartDashboard.getNumber("AssistReefDistance", 1.0)
            || VisionSubsystem.DistanceToProcessor()
                > SmartDashboard.getNumber("AssistProcessorDistance", 1.0));
  }
}
