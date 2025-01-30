package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorNextPosition extends Command {
  private ElevatorSubsystem m_subsystem;
  private double ElevatorPosition;
  private int position;

  public ElevatorNextPosition(ElevatorSubsystem subsystem) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    ElevatorPosition = m_subsystem.getPosition();
    if (m_subsystem.levels[0] >= ElevatorPosition - 10
        && m_subsystem.levels[0] <= ElevatorPosition + 10) {
      position = 2;
      m_subsystem.setLevel(position);
    } else if (m_subsystem.levels[1] >= ElevatorPosition - 10
        && m_subsystem.levels[1] <= ElevatorPosition + 10) {
      position = 3;
      m_subsystem.setLevel(position);
    } else if (m_subsystem.levels[2] >= ElevatorPosition - 10
        && m_subsystem.levels[2] <= ElevatorPosition + 10) {
      position = 4;
      m_subsystem.setLevel(position);
    } else if (m_subsystem.levels[3] >= ElevatorPosition - 10
        && m_subsystem.levels[3] <= ElevatorPosition + 10) {
      position = 1;
      m_subsystem.setLevel(position);
    }
  }

  @Override
  public boolean isFinished() {
    return m_subsystem.isFinished(position);
  }
}
