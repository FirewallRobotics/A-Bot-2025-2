package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorBWD extends Command {
  private final ElevatorSubsystem m_subsystem;
  private double axis;

  public ElevatorBWD(ElevatorSubsystem subsystem, double axis) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
    this.axis = axis;
  }

  @Override
  public void execute() {
    m_subsystem.setSpeed(-axis);
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
