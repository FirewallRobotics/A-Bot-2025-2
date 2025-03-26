package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralHoldAngleSubsystem;
import java.util.logging.Level;
import java.util.logging.Logger;

public class ArmLevel3 extends Command {
  private CoralHoldAngleSubsystem coralHold;

  public ArmLevel3(CoralHoldAngleSubsystem c_Subsystem) {
    coralHold = c_Subsystem;
    addRequirements(c_Subsystem);
  }

  @Override
  public void execute() {
    coralHold.setLevel(3);

    if (coralHold.atLevel(3)) {

      // Logger.getGlobal().log(Level.INFO, "found level");
      coralHold.stopTilt();
    }
  }

  @Override
  public boolean isFinished() {
    if (coralHold.atLevel(3)) {
      Logger.getGlobal().log(Level.INFO, "found level");
      coralHold.stopTilt();
      return true;
    }
    return false;
  }
}
