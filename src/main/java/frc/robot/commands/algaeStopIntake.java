package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeSubsystem;

public class algaeStopIntake extends Command {

  AlgaeSubsystem algaeSubsystem;

  public algaeStopIntake(AlgaeSubsystem mSubsystem) {
    algaeSubsystem = mSubsystem;
  }

  @Override
  public void initialize() {
    algaeSubsystem.stop();
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
