package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralHoldAngleSubsystem;

public class WristStop extends Command {
  private CoralHoldAngleSubsystem coralHold;

  public WristStop(CoralHoldAngleSubsystem ca_Subsystem) {
    coralHold = ca_Subsystem;
  }

  @Override
  public void initialize() {
    coralHold.stopTilt();
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
