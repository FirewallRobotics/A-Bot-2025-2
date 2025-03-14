package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralHoldSubsystem;

public class stopCoralIntake extends Command {

  CoralHoldSubsystem coralHoldSubsystem;

  public stopCoralIntake(CoralHoldSubsystem mSubsystem) {
    coralHoldSubsystem = mSubsystem;
  }

  @Override
  public void initialize() {
    coralHoldSubsystem.stop();
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
