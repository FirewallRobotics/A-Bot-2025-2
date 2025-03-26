package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralHoldAngleSubsystem;

public class ArmLevel3 extends Command {
  private CoralHoldAngleSubsystem coralHold;

  public ArmLevel3(CoralHoldAngleSubsystem c_Subsystem) {
    coralHold = c_Subsystem;
    addRequirements(c_Subsystem);
  }

  @Override
  public void initialize() {
    coralHold.setLevel();
  }

  @Override
  public boolean isFinished() {
    return ((coralHold.getFinalPos() - 2) >= coralHold.getPosition())
        && ((coralHold.getFinalPos() + 2) <= coralHold.getPosition());
  }
}
