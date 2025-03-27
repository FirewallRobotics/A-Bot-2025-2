package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralHoldAngleSubsystem;

public class WristToL1 extends Command {
  private CoralHoldAngleSubsystem coralHold;

  public WristToL1(CoralHoldAngleSubsystem ca_Subsystem) {
    coralHold = ca_Subsystem;
  }

  @Override
  public void initialize() {
    coralHold.tiltedDown();
  }

  @Override
  public void end(boolean inter) {
    coralHold.stopTilt();
  }

  @Override
  public boolean isFinished() {
    if (coralHold.getEncoder() >= 0) {
      return coralHold.getEncoder() >= 3.5;
    } else {
      return coralHold.getEncoder() <= -3.5;
    }
  }
}
