package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorStop extends Command {
  private ElevatorSubsystem m_subsystem;
  private int divider;

  public ElevatorStop(ElevatorSubsystem subsystem) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    /*
    if (ElevatorSubsystem.levels.length - 1 == SmartDashboard.getNumber("ElevatorPos", 0)) {
      m_subsystem.setLevel(0);
    } else {
      m_subsystem.setLevel((int) SmartDashboard.getNumber("ElevatorPos", 0) + 1);
    }
      */
    divider = 20;
  }

  @Override
  public void execute() {
    m_subsystem.setSpeed((m_subsystem.getSpeed() / 2));
    divider -= 1;
  }

  @Override
  public boolean isFinished() {
    return divider == 0;
  }
}
