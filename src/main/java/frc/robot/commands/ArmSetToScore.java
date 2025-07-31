package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralWristSubsystem;

public class ArmSetToScore extends Command {
  private CoralWristSubsystem wristSubsystem;
  private boolean needsMovement;

  public ArmSetToScore(CoralWristSubsystem cw_Subsystem) {
    wristSubsystem = cw_Subsystem;

    addRequirements(cw_Subsystem);
  }

  @Override
  public void initialize() {
    needsMovement = !wristSubsystem.atLevel(2);
  }

  @Override
  public void execute() {
    if (needsMovement) {
      wristSubsystem.goToCoralWristLevel(2);
    }
  }

  @Override
  public boolean isFinished() {
    if (wristSubsystem.atLevel(2) && needsMovement) {
      wristSubsystem.stopWrist();
      return true;
    } else if (!needsMovement) {
      return true;
    }

    return false;
  }
}
