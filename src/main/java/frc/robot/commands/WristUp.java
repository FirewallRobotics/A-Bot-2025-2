package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralHoldAngleSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import java.util.logging.Level;
import java.util.logging.Logger;

public class WristUp extends Command {
  private CoralHoldAngleSubsystem coralHold;

  public WristUp(CoralHoldAngleSubsystem ca_Subsystem) {
    coralHold = ca_Subsystem;

    addRequirements(ca_Subsystem);
  }

  @Override
  public void initialize() {
    Logger.getGlobal().log(Level.INFO, "UP: " + coralHold.getPosition());
    coralHold.tiltUp();
    // if (elevator.getPositionEncoder() != 50) {
    //   coralHold.tiltUp();
    // }
  }

  @Override
  public void execute() {
    Logger.getGlobal().log(Level.INFO, "UP: " + coralHold.getPosition());
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
