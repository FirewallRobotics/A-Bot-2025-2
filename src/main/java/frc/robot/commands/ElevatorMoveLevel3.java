package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

/** An example command that uses an example subsystem. */
public class ElevatorMoveLevel3 extends Command {
  private final ElevatorSubsystem m_subsystem;

  /**
   * Creates a new ElevatorMoveLevel3.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ElevatorMoveLevel3(ElevatorSubsystem subsystem) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  @Override
  public void initialize(){
    m_subsystem.setLevel(3);
  }

  @Override
  public boolean isFinished(){
    return m_subsystem.isFinished(3);
  }
}
