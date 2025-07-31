package frc.robot.commands;

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
  }

  public void execute() {
    if (needsMovement) {
      wristSubsystem.goToCoralWristLevel(1);
    }
    
  }

  public boolean isFinished() {
    if (wristSubsystem.atLevel(1) && needsMovement) {
      wristSubsystem.stopWrist();
      return true;
    } else if(!needsMovement){
      return true;
    }

    return false;
  }
}
