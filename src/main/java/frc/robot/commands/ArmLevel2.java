package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralHoldAngleSubsystem;
import java.util.logging.Level;
import java.util.logging.Logger;

public class ArmLevel2 extends Command {
  private CoralHoldAngleSubsystem coralHold;

  public ArmLevel2(CoralHoldAngleSubsystem c_Subsystem) {
    coralHold = c_Subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(c_Subsystem);
  }

  public void execute() {
    coralHold.setLevel(1);

    if (coralHold.atLevel(1)) {

      // Logger.getGlobal().log(Level.INFO, "found level");
      coralHold.stopTilt();
    }
  }

  @Override
  public boolean isFinished() {
    if (coralHold.atLevel(1)) {
      Logger.getGlobal().log(Level.INFO, "found level");
      coralHold.stopTilt();
      return true;
    }
    return false;
  }
}
