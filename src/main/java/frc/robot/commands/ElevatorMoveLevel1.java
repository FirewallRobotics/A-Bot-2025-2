package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

/** An example command that uses an example subsystem. */
public class ElevatorMoveLevel1 extends Command {
  private final ElevatorSubsystem m_subsystem;

  /**
   * Creates a new ElevatorMoveLevel1.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ElevatorMoveLevel1(ElevatorSubsystem subsystem) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  @Override
  public void initialize(){
    m_subsystem.setLevel(1);
  }

  @Override
  public boolean isFinished(){
    return m_subsystem.isFinished(1);
  }
}
