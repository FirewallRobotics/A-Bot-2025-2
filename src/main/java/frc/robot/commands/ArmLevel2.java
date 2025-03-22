package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralHoldAngleSubsystem;

public class ArmLevel2 extends Command {
  private CoralHoldAngleSubsystem coralHold;

  public ArmLevel2(CoralHoldAngleSubsystem c_Subsystem) {
    coralHold = c_Subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(c_Subsystem);
  }

  @Override
  public void initialize() {
    coralHold.setLevel(2);
  }

  @Override
  public boolean isFinished() {
    return coralHold.isFinished(2);
  }
}
