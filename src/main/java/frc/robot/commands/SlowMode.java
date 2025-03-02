package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class SlowMode extends Command {

  @Override
  public void initialize() {
    RobotContainer.drivebase.setMaxSpeed(1);
    SmartDashboard.putBoolean("SlowMode", true);
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
