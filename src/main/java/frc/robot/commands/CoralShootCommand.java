package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CoralHoldSubsystem;

public class CoralShootCommand extends Command {
  public final CoralHoldSubsystem m_Coral;
  private RobotContainer robotContainer;

  public CoralShootCommand(CoralHoldSubsystem c_Subsystem) {
    m_Coral = c_Subsystem;
  }

  public CoralShootCommand(CoralHoldSubsystem c_Subsystem, RobotContainer robotContainer) {
    m_Coral = c_Subsystem;
    this.robotContainer = robotContainer;
  }

  // Called when the command is first scheduled
  @Override
  public void initialize() {
    if (robotContainer != null) {
      // robotContainer.repeatWristDown.cancel();
    }
    // RobotContainer.coralHoldAngleSubsystem.LPosition();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Coral.shoot();
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
