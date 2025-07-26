package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralWristSubsystem;

public class ArmSetToCoralAccept extends Command {
  private CoralWristSubsystem wristSubsystem;

  public ArmSetToCoralAccept(CoralWristSubsystem cw_Subsystem) {
    wristSubsystem = cw_Subsystem;

    addRequirements(cw_Subsystem);
  }

  @Override
  public void execute() {
    wristSubsystem.goToCoralWristLevel(3);

    if (wristSubsystem.atLevel(3)) {
      wristSubsystem.stopWrist();
    }
  }

  @Override
  public boolean isFinished() {
    if (wristSubsystem.atLevel(3)) {
      wristSubsystem.stopWrist();
      return true;
    }
    return false;
  }
}
