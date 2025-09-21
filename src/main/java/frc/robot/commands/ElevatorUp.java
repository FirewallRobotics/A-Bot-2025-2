package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorCoralSubsystem;

public class ElevatorUp extends Command {
  private ElevatorCoralSubsystem elevatorSubsystem;

  public ElevatorUp(ElevatorCoralSubsystem e_Subsystem) {
    elevatorSubsystem = e_Subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(e_Subsystem);
  }

  @Override
  public void initialize() {
    elevatorSubsystem.ElevatorUp();
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
