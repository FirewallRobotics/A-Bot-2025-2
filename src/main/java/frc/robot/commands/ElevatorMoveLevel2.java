package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorCoralSubsystem;

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
    needsMovement = !elevatorSubsystem.atLevel(2);
  }

  @Override
  public void execute() {

    if (needsMovement && !elevatorSubsystem.atLevel(2)) {
      elevatorSubsystem.ToCoralLevel(2);
    }
  }

  @Override
  public boolean isFinished() {
    if (elevatorSubsystem.atLevel(2) && needsMovement) {
      elevatorSubsystem.ElevatorStop();

      return true;
    } else if (!needsMovement) {
      // elevatorSubsystem.ElevatorStop(true);
      return true;
    }
    return false;
  }
}
