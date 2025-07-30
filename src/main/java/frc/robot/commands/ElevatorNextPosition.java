package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorCoralSubsystem;

public class ElevatorNextPosition extends Command {
  private ElevatorCoralSubsystem m_subsystem;

  public ElevatorNextPosition(ElevatorCoralSubsystem subsystem) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    // if (ElevatorSubsystemConstants.elevatorLevels.length - 1 ==
    // SmartDashboard.getNumber("ElevatorPos", 0)) {
    //   m_subsystem.setLevel(0);
    // } else {
    //   m_subsystem.setLevel((int) SmartDashboard.getNumber("ElevatorPos", 0) + 1);
    // }
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
