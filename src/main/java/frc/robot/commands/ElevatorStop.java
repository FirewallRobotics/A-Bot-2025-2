package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import java.util.logging.Level;
import java.util.logging.Logger;

public class ElevatorStop extends Command {
  private ElevatorSubsystem m_subsystem;

  public ElevatorStop(ElevatorSubsystem subsystem) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    m_subsystem.stop();
    Logger.getGlobal().log(Level.INFO, "Stopping Elevator");
  }

  @Override
  public void execute() {}

  @Override
  public boolean isFinished() {
    return true;
  }
}
