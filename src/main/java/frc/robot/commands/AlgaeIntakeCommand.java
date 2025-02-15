package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeSubsystem;

public class AlgaeIntakeCommand extends Command {
  public final AlgaeSubsystem m_Algae;

  public AlgaeIntakeCommand(AlgaeSubsystem c_Subsystem) {
    m_Algae = c_Subsystem;
  }

  // Called when the command is first scheduled
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Algae.intake();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Algae.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Should come back to this
    return false;
  }
}
