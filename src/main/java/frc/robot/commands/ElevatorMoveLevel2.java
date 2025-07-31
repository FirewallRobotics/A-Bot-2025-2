package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorCoralSubsystem;
import java.util.logging.Level;
import java.util.logging.Logger;

/** An example command that uses an example subsystem. */
public class ElevatorMoveLevel2 extends Command {
  private final ElevatorCoralSubsystem elevatorSubsystem;

  private boolean needsMovement;

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
  public void initialize() {
    Logger.getGlobal().log(Level.INFO, "initalized");
    needsMovement = !elevatorSubsystem.atLevel(2);
  }

  @Override
  public void execute() {
    
    if (needsMovement && !elevatorSubsystem.atLevel(2)) {
      Logger.getGlobal().log(Level.INFO, "Needs to move: " + elevatorSubsystem.getPositionEncoder());
      elevatorSubsystem.ToCoralLevel(2);
    } else {
      Logger.getGlobal().log(Level.INFO, "doesn't need to move");
    }
  }

  @Override
  public boolean isFinished() {
    if (elevatorSubsystem.atLevel(2) && needsMovement) {
      Logger.getGlobal().log(Level.INFO, "Elevator found level");
      elevatorSubsystem.ElevatorStop();

      return true;
    } else if (!needsMovement) {
      // elevatorSubsystem.ElevatorStop(true);
      return true;
    }
    Logger.getGlobal().log(Level.INFO, "Elevator fuck up");
    return false;
  }
}
