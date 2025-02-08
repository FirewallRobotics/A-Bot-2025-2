package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.robot.Constants.ElevatorSubsystemConstants;

public class RobotMath {

  public static class Elevator {

    /**
     * Convert {@link Distance} into {@link Angle}
     *
     * @param distance Distance, usually Meters.
     * @return {@link Angle} equivalent to rotations of the motor.
     */
    public static Angle convertDistanceToRotations(Distance distance) {
      return Rotations.of(
          distance.in(Meters)
              / (ElevatorSubsystemConstants.kElevatorDrumRadius * 2 * Math.PI)
              * ElevatorSubsystemConstants.kElevatorGearing);
    }

    /**
     * Convert {@link Angle} into {@link Distance}
     *
     * @param rotations Rotations of the motor
     * @return {@link Distance} of the elevator.
     */
    public static Distance convertRotationsToDistance(Angle rotations) {
      return Meters.of(
          (rotations.in(Rotations) / ElevatorSubsystemConstants.kElevatorGearing)
              * (ElevatorSubsystemConstants.kElevatorDrumRadius * 2 * Math.PI));
    }
  }
}
