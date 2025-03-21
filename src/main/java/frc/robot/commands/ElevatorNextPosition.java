package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorNextPosition extends Command {
  private ElevatorSubsystem m_subsystem;

  public ElevatorNextPosition(ElevatorSubsystem subsystem) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    if (ElevatorSubsystem.levels.length - 1 == SmartDashboard.getNumber("ElevatorPos", 0)) {
      m_subsystem.setLevel(0);
    } else {
      m_subsystem.setLevel((int) SmartDashboard.getNumber("ElevatorPos", 0) + 1);
    }
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
