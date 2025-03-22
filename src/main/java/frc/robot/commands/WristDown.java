package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralHoldAngleSubsystem;

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
  public boolean isFinished() {
    return true;
  }
}
