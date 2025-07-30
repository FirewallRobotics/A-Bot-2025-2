package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorCoralSubsystem;
import java.util.logging.Level;
import java.util.logging.Logger;

public class ElevatorDown extends Command {
  private ElevatorCoralSubsystem elevatorSubsystem;

  public ElevatorDown(ElevatorCoralSubsystem e_Subsystem) {
    elevatorSubsystem = e_Subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(e_Subsystem);
  }

  @Override
  public void initialize() {
    Logger.getGlobal().log(Level.INFO, "Manual Elevator down");
    elevatorSubsystem.ElevatorDown();
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
