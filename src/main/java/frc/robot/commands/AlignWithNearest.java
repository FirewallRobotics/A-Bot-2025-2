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

    // if we are close enough to the reef
    if (VisionSubsystem.DistanceToReef() < SmartDashboard.getNumber("AssistMinDistance", 10)) {
      LimelightHelpers.setPipelineIndex("", 2);
      DoubleSupplier zero = () -> 0.0;
      while (LimelightHelpers.getTX("") > SmartDashboard.getNumber("AssistLossRange", 1.0)) {
        DoubleSupplier movespeed = () -> SmartDashboard.getNumber("AutoMoveSpeed", 1.0);
        drivebase.driveCommand(zero, movespeed, null);
      }
      while (LimelightHelpers.getTX("") < -SmartDashboard.getNumber("AssistLossRange", 1.0)) {
        DoubleSupplier movespeed = () -> -SmartDashboard.getNumber("AutoMoveSpeed", 1.0);
        drivebase.driveCommand(zero, movespeed, null);
      }
      while (VisionSubsystem.DistanceToReef()
          > SmartDashboard.getNumber("AssistReefDistance", 1.0)) {
        DoubleSupplier movespeed = () -> -SmartDashboard.getNumber("AutoMoveSpeed", 1.0);
        drivebase.driveCommand(movespeed, zero, null);
      }
      drivebase.driveCommand(zero, zero, null);
    }

    // if we are close enough to the processor
    if (VisionSubsystem.DistanceToProcessor() < SmartDashboard.getNumber("AssistMinDistance", 10)) {
      LimelightHelpers.setPipelineIndex("", 3);
      DoubleSupplier zero = () -> 0.0;
      while (LimelightHelpers.getTX("") > SmartDashboard.getNumber("AssistLossRange", 1.0)) {
        DoubleSupplier movespeed = () -> SmartDashboard.getNumber("AutoMoveSpeed", 1.0);
        drivebase.driveCommand(zero, movespeed, null);
      }
      while (LimelightHelpers.getTX("") < -SmartDashboard.getNumber("AssistLossRange", 1.0)) {
        DoubleSupplier movespeed = () -> -SmartDashboard.getNumber("AutoMoveSpeed", 1.0);
        drivebase.driveCommand(zero, movespeed, null);
      }
      while (VisionSubsystem.DistanceToProcessor()
          > SmartDashboard.getNumber("AssistProcessorDistance", 1.0)) {
        DoubleSupplier movespeed = () -> -SmartDashboard.getNumber("AutoMoveSpeed", 1.0);
        drivebase.driveCommand(movespeed, zero, null);
      }
      drivebase.driveCommand(zero, zero, null);
    }
  }

  @Override
  public boolean isFinished() {
    double temp = LimelightHelpers.getTX("");
    return temp < SmartDashboard.getNumber("AssistLossRange", 1.0)
        && temp > -SmartDashboard.getNumber("AssistLossRange", 1.0);
  }
}
