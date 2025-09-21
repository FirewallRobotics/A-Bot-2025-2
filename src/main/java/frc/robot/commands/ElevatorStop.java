package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorCoralSubsystem;

public class ElevatorStop extends Command {
  private ElevatorCoralSubsystem elevatorSubsystem;

  public ElevatorStop(ElevatorCoralSubsystem e_Subsystem) {
    elevatorSubsystem = e_Subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(e_Subsystem);
  }

  @Override
  public void initialize() {
    elevatorSubsystem.ElevatorStop();
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
