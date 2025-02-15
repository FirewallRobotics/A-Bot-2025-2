package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralHoldSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class CoralIntakeCommand extends Command {
  public final CoralHoldSubsystem m_Coral;
  public final ElevatorSubsystem m_ElevatorSubsystem;

  public CoralIntakeCommand(CoralHoldSubsystem c_Subsystem, ElevatorSubsystem c_ElevatorSubsystem) {
    m_Coral = c_Subsystem;
    m_ElevatorSubsystem = c_ElevatorSubsystem;
  }

  // Called when the command is first scheduled
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_ElevatorSubsystem.setLevel(0);
    m_Coral.intake();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Coral.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Should come back to this
    return false;
  }
}
