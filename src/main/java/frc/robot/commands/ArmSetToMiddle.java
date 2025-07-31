package frc.robot.commands;

import java.util.logging.Level;
import java.util.logging.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralWristSubsystem;

public class ArmSetToMiddle extends Command {
  private final CoralWristSubsystem wristSubsystem;
  private boolean needsMovement;

  public ArmSetToMiddle(CoralWristSubsystem cw_Subsystem) {
    wristSubsystem = cw_Subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(cw_Subsystem);
  }
  @Override
  public void initialize() {
    needsMovement = !wristSubsystem.atLevel(1);
    Logger.getGlobal().log(Level.INFO, "Needs move: " + needsMovement);
  }

  public void execute() {
    if (needsMovement) {
      
      wristSubsystem.goToCoralWristLevel(1);
    }
    
  }

  public boolean isFinished() {
    if (wristSubsystem.atLevel(1) && needsMovement) {
      Logger.getGlobal().log(Level.INFO, " Wrist At setpoint");
      wristSubsystem.stopWrist();
      return true;
    } else if(!needsMovement){
      return true;
    }
    Logger.getGlobal().log(Level.INFO, "Not at setpoint");
    return false;
  }
}
