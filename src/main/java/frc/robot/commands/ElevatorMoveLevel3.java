package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorCoralSubsystem;

/** An example command that uses an example subsystem. */
public class ElevatorMoveLevel3 extends Command {
  private final ElevatorCoralSubsystem elevatorSubsystem;
  private boolean movementNeeded;

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
  public void initialize() {
    // If it is at the level it wants, it will return false
    // If it is not at the level it wants, it will return true
    movementNeeded = !elevatorSubsystem.atLevel(3);
  }

  @Override
  public void execute() {
    if (movementNeeded) {
      elevatorSubsystem.ToCoralLevel(3);
    }
  }

  @Override
  public boolean isFinished() {

    if (elevatorSubsystem.atLevel(3) && movementNeeded) {

      elevatorSubsystem.ElevatorStop();

      return true;
    } else if (!movementNeeded) {
      // elevatorSubsystem.ElevatorStop(true);
      return true;
    }
    return false;
  }
}
