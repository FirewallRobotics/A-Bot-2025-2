package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class ArmLower extends Command {
  private ClimberSubsystem m_subsystem;

  public ArmLower(ClimberSubsystem subsystem) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // hold button down and when released, climber stops

  @Override
  public void initialize() {
    // :)
  }

  @Override
  public void execute() {
    m_subsystem.StartRetracting();
  }

  @Override
  public void end(boolean interrupted) {
    m_subsystem.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
