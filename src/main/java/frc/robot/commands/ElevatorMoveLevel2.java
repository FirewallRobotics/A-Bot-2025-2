package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorCoralSubsystem;
import java.util.logging.Level;
import java.util.logging.Logger;

/** An example command that uses an example subsystem. */
public class ElevatorMoveLevel2 extends Command {
  private final ElevatorCoralSubsystem elevatorSubsystem;

  /**
   * Creates a new ElevatorMoveLevel2.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ElevatorMoveLevel2(ElevatorCoralSubsystem e_Subsystem) {
    elevatorSubsystem = e_Subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(e_Subsystem);
  }

  @Override
  public void execute() {

    if (elevatorSubsystem.atLevel(2)) {

      // Logger.getGlobal().log(Level.INFO, "found level");
      elevatorSubsystem.ElevatorStop();
    } else {
      elevatorSubsystem.ToCoralLevel(2);
    }
  }

  @Override
  public boolean isFinished() {
    if (elevatorSubsystem.atLevel(2)) {
      Logger.getGlobal().log(Level.INFO, "found level");
      elevatorSubsystem.ElevatorStop();

      return true;
    }
    return false;
  }
}
