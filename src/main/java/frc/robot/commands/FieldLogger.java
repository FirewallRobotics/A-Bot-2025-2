package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.VisionSubsystem;

public class FieldLogger extends Command {

  private final VisionSubsystem m_VisionSubsystem;

  public FieldLogger(VisionSubsystem v_Subsystem) {

    m_VisionSubsystem = v_Subsystem;
  }

  public void LogFieldPosition() {
    m_VisionSubsystem.getPose3d();
  }
}
