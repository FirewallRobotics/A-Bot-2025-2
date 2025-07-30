package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorCoralSubsystem;
import java.util.logging.Level;
import java.util.logging.Logger;

/** An example command that uses an example subsystem. */
public class ElevatorMoveLevel3 extends Command {
  private final ElevatorCoralSubsystem elevatorSubsystem;

  /**
   * Creates a new ElevatorMoveLevel3.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ElevatorMoveLevel3(ElevatorCoralSubsystem e_Subsystem) {
    elevatorSubsystem = e_Subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(e_Subsystem);
  }

  @Override
  public void execute() {
    elevatorSubsystem.ToCoralLevel(3);

    if (elevatorSubsystem.atLevel(3)) {

      // Logger.getGlobal().log(Level.INFO, "found level");
      elevatorSubsystem.ElevatorStop();
    }
  }

  @Override
  public boolean isFinished() {

    if (elevatorSubsystem.atLevel(3)) {
      Logger.getGlobal().log(Level.INFO, "found level");
      elevatorSubsystem.ElevatorStop();
      return true;
    }
    return false;
  }
}
