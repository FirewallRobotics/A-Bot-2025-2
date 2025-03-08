package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralHoldAngleSubsystem;

public class WristUp extends Command {
  private CoralHoldAngleSubsystem coralHold;

  public WristUp(CoralHoldAngleSubsystem ca_Subsystem) {
    coralHold = ca_Subsystem;
  }

  @Override
  public void execute() {
    coralHold.tiltUp();
  }

  @Override
  public void end(boolean interrupted) {
    coralHold.stopTilt();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
