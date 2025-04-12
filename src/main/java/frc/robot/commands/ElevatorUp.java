package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import java.util.logging.Level;
import java.util.logging.Logger;

public class ElevatorUp extends Command {
  private ElevatorSubsystem m_subsystem;

  private double position;

  public ElevatorUp(ElevatorSubsystem subsystem, double speed) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
    this.position = speed;
  }

  @Override
  public void initialize() {
    if (!SmartDashboard.getBoolean("Kid-Mode", false)) {
      Logger.getGlobal().log(Level.INFO, "Moving Elevator With PWR: " + position);
      m_subsystem.setSpeed(-position);
    } else {
      Logger.getGlobal()
          .log(Level.INFO, "Moving Elevator(In kid mode) With PWR: " + (position / 2));
      m_subsystem.setSpeed(-(position / 2));
    }
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
