package frc.robot.commands;

import java.util.logging.Level;
import java.util.logging.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorUp extends Command {
  private ElevatorSubsystem m_subsystem;

  private double position;

  public ElevatorUp(ElevatorSubsystem subsystem, double speed) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
    this.position = speed;
  }

  @Override
  public void initialize() {
    Logger.getGlobal().log(Level.INFO, "Moving Elevator With PWR: " + position);
    m_subsystem.setSpeed(-position);
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
