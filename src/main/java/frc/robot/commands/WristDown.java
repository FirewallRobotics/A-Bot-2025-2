package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralWristSubsystem;

public class WristDown extends Command {
  private final CoralWristSubsystem wristSubsystem;

  public WristDown(CoralWristSubsystem cw_Subsystem) {
    wristSubsystem = cw_Subsystem;
  }

  @Override
  public void initialize() {
    wristSubsystem.goDown();
  }

  @Override
  public boolean isFinished() {

    return true;
  }
}
