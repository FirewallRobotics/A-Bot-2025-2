package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import java.util.logging.Level;
import java.util.logging.Logger;

/** An example command that uses an example subsystem. */
public class ElevatorMoveLevel2 extends Command {
  private final ElevatorSubsystem m_subsystem;

  /**
   * Creates a new ElevatorMoveLevel2.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ElevatorMoveLevel2(ElevatorSubsystem subsystem) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  @Override
  public void execute() {
    m_subsystem.goToCoralLevel(2);

    if (m_subsystem.atLevel(2)) {

      // Logger.getGlobal().log(Level.INFO, "found level");
      m_subsystem.stop();
    }
  }

  @Override
  public boolean isFinished() {
    if (m_subsystem.atLevel(2)) {
      Logger.getGlobal().log(Level.INFO, "found level");
      m_subsystem.stop();
      return true;
    }
    return false;
  }
}
