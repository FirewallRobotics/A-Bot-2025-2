package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralWristSubsystem;

public class ArmSetToScore extends Command {
  private CoralWristSubsystem wristSubsystem;

  public ArmSetToScore(CoralWristSubsystem cw_Subsystem) {
    wristSubsystem = cw_Subsystem;

    addRequirements(cw_Subsystem);
  }

  @Override
  public void execute() {
    wristSubsystem.goToCoralWristLevel(2);

    if (wristSubsystem.atLevel(2)) {

      // Logger.getGlobal().log(Level.INFO, "found level");
      wristSubsystem.stopWrist();
    }
  }

  @Override
  public boolean isFinished() {
    if (wristSubsystem.atLevel(2)) {
      wristSubsystem.stopWrist();
      return true;
    }

    return false;
  }
}
