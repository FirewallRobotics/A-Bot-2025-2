package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeSubsystem;

public class AlgaeShootCommand extends Command {
  public final AlgaeSubsystem m_Algae;

  public AlgaeShootCommand(AlgaeSubsystem c_Subsystem) {
    m_Algae = c_Subsystem;
  }

  // Called when the command is first scheduled
  @Override
  public void initialize() {
    m_Algae.shoot();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Should come back to this
    return true;
  }
}
