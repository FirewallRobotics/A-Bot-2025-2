package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralHoldAngleSubsystem;

public class ArmLower extends Command {
  private CoralHoldAngleSubsystem m_coral;

  public ArmLower(CoralHoldAngleSubsystem subsystem) {
    m_coral = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_coral.tiltedDown();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_coral.stopTilt();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Should come back to this
    return false;
  }
}
