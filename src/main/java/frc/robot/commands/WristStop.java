package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralWristSubsystem;
import java.util.logging.Level;
import java.util.logging.Logger;

public class WristStop extends Command {
  private final CoralWristSubsystem wristSubsystem;

  public WristStop(CoralWristSubsystem cw_Subsystem) {
    wristSubsystem = cw_Subsystem;
  }

  @Override
  public void initialize() {
    wristSubsystem.stopWrist();
    Logger.getGlobal().log(Level.INFO, "Wrist manual control: stopping");
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
