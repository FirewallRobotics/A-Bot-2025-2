package frc.robot.commands;

import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;


public class ArmRaise {
    private ClimberSubsystem m_subsystem;

    public ArmRaise(ClimberSubsystem subsystem) {
        m_subsystem = subsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }
}