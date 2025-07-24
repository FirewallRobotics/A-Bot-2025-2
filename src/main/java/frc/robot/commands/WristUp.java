package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralHoldAngleSubsystem;
import java.util.logging.Level;
import java.util.logging.Logger;

public class WristUp extends Command {
  private CoralHoldAngleSubsystem coralHold;

  public WristUp(CoralHoldAngleSubsystem ca_Subsystem) {
    coralHold = ca_Subsystem;

    addRequirements(ca_Subsystem);
  }

  @Override
  public void initialize() {
    coralHold.tiltUp();
    // if (elevator.getPositionEncoder() != 50) {
    //   coralHold.tiltUp();
    // }
  }

  @Override
  public void execute() {
    SmartDashboard.putNumber("WhereTheHeckThingsAre", coralHold.getEncoder());
    Logger.getGlobal().log(Level.INFO, "Wrist going up: " + coralHold.getEncoder());
  }

  @Override
  public boolean isFinished() {
    Logger.getGlobal().log(Level.INFO, "Wrist going up: " + coralHold.getEncoder());
    return true;
  }
}
