package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralHoldAngleSubsystem;

public class AngleUpDown extends Command {
    private final CoralHoldAngleSubsystem coralHoldAngleSubsystemSubsystem;
    private final double speed;

    public AngleUpDownCommand(CoralHoldAngleSubsystem coralHoldAngleSubsystem, double speed) {
        this.CoralHoldAngleSubsystemSubsystem = coralHoldAngleSubsystemSubsystem;
        this.speed = speed;
        addRequirements(coralHoldAngleSubsystem); // Declare subsystem dependencies
    }

    @Override
    public void initialize() {
        // Set the motor speed based on the direction (up or down)
        if (speed > 0) {
            CoralHoldAngleSubsystem.moveUp();
        } else if (speed < 0) {
            CoralHoldAngleSubsystem.moveDown();
        }
    }

    @Override
    public void end(boolean interrupted) {
        CoralHoldAngleSubsystem.stop(); // Stop the motor when the command ends
    }

    @Override
    public boolean isFinished() {
        return false; // Command continues until interrupted
    }
}