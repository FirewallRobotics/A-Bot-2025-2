package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralHoldAngleSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class WristUp extends Command {
  private CoralHoldAngleSubsystem coralHold;
  private ElevatorSubsystem elevator;

  public WristUp(CoralHoldAngleSubsystem ca_Subsystem, ElevatorSubsystem e_Subsystem) {
    coralHold = ca_Subsystem;
    elevator = e_Subsystem;

    addRequirements(ca_Subsystem, e_Subsystem);
  }

  @Override
  public void initialize() {
    coralHold.tiltUp();
    // if (elevator.getPositionEncoder() != 50) {
    //   coralHold.tiltUp();
    // }
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
