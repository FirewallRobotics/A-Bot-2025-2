package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorPrevPosition extends Command {
  private ElevatorSubsystem m_subsystem;

  // private int position;

  public ElevatorPrevPosition(ElevatorSubsystem subsystem) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    /*
    if (0 == SmartDashboard.getNumber("ElevatorPos", 0)) {
      m_subsystem.setLevel(ElevatorSubsystem.levels.length - 1);
    } else {
      m_subsystem.setLevel((int) SmartDashboard.getNumber("ElevatorPos", 0) - 1);
    }
      */
    m_subsystem.setSpeed(-0.6);
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
