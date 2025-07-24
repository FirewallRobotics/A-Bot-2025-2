package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralHoldAngleSubsystem;
import java.util.logging.Level;
import java.util.logging.Logger;

public class WristDown extends Command {
  private CoralHoldAngleSubsystem coralHold;

  public WristDown(CoralHoldAngleSubsystem ca_Subsystem) {
    coralHold = ca_Subsystem;
  }

  @Override
  public void initialize() {
    coralHold.tiltedDown();
  }

  @Override
  public void execute() {
    SmartDashboard.putNumber("WhereTheHeckThingsAre", coralHold.getEncoder());
    Logger.getGlobal().log(Level.INFO, "Wrist going down: " + coralHold.getEncoder());
  }

  @Override
  public boolean isFinished() {
    Logger.getGlobal().log(Level.INFO, "Wrist going down: " + coralHold.getEncoder());
    return true;
  }
}
