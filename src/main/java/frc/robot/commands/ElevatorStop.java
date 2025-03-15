package frc.robot.commands;

import java.util.logging.Level;
import java.util.logging.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Main;
import frc.robot.subsystems.ElevatorSubsystem;

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
