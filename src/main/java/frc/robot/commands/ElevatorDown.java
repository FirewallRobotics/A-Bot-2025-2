package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorDown extends Command {
  private ElevatorSubsystem m_subsystem;

  private double position;

  public ElevatorDown(ElevatorSubsystem subsystem, double position) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
    this.position = position;
  }

  @Override
  public void initialize() {
    m_subsystem.setSpeed(0.6);
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
