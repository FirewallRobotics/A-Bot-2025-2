package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import java.util.logging.Level;
import java.util.logging.Logger;

public class ElevatorDown extends Command {
  private ElevatorSubsystem m_subsystem;

  private double position;

  public ElevatorDown(ElevatorSubsystem subsystem, double speed) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
    this.position = speed;
  }

  @Override
  public void initialize() {
    if (!SmartDashboard.getBoolean("Kid-Mode", false)) {
      Logger.getGlobal().log(Level.INFO, "Moving Elevator Down With PWR: " + position);
      m_subsystem.setSpeed(position);
    } else {
      Logger.getGlobal()
          .log(Level.INFO, "Moving Elevator(Kid mode) Down With PWR: " + (position / 1.5));
      m_subsystem.setSpeed((position / 1.5));
    }
  }

  @Override
  public void execute() {}

  @Override
  public boolean isFinished() {
    return true;
  }
}
