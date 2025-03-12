package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorUp extends Command {
  private ElevatorSubsystem m_subsystem;

  private double position;
  private int divider;

  public ElevatorUp(ElevatorSubsystem subsystem, double position) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
    this.position = position;
  }

  @Override
  public void initialize() {
    divider = 20;
  }

  @Override
  public void execute() {
    if (position != 0) {
      m_subsystem.setSpeed(-(position / divider));
      divider -= 1;
    } else {
      m_subsystem.setSpeed((m_subsystem.getSpeed() / 2));
      divider -= 1;
    }
  }

  @Override
  public boolean isFinished() {
    return divider == 0;
  }
}
