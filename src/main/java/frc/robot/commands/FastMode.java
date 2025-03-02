package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class FastMode extends Command {
  @Override
  public void initialize() {
    RobotContainer.drivebase.setMaxSpeed(5);
    SmartDashboard.putBoolean("SlowMode", false);
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
