package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralWristSubsystem;

public class WristUp extends Command {
  private final CoralWristSubsystem wristSubsystem;

  public WristUp(CoralWristSubsystem cw_Subsystem) {
    wristSubsystem = cw_Subsystem;
  }

  @Override
  public void initialize() {
    wristSubsystem.goUp();
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
