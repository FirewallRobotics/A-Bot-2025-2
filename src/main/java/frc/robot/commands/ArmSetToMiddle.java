package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralWristSubsystem;

public class ArmSetToMiddle extends Command {
  private CoralWristSubsystem wristSubsystem;

  public ArmSetToMiddle(CoralWristSubsystem cw_Subsystem) {
    wristSubsystem = cw_Subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(cw_Subsystem);
  }

  public void execute() {
    wristSubsystem.goToCoralWristLevel(1);

    if (wristSubsystem.atLevel(1)) {

      // Logger.getGlobal().log(Level.INFO, "found level");
      wristSubsystem.stopWrist();
    }
  }

  public boolean isFinished() {
    if (wristSubsystem.atLevel(1)) {
      wristSubsystem.stopWrist();
      return true;
    }

    return false;
  }
}
