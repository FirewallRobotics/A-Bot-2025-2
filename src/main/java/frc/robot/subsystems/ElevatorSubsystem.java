package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
  private final MotorController motor;
  private final Encoder encoder;

  // Elevator levels in encoder ticks
  private final int[] levels = {0, 1000, 2000, 3000, 4000};

  public ElevatorSubsystem() {
    motor = new PWMSparkMax(0); // Assign motor controller port
    encoder = new Encoder(0, 1); // Assign encoder ports
  }

  public void setLevel(int level) {
    if (level < 0 || level >= levels.length) {
      System.out.println("Invalid level: " + level);
      return;
    }
    moveToPosition(levels[level]);
  }

  private void moveToPosition(int position) {
    double speed = calculateSpeed(position);
    motor.set(speed);
  }

  private double calculateSpeed(int targetPosition) {
    int currentPosition = encoder.get();
    int error = targetPosition - currentPosition;

    // Simple proportional control for example purposes
    double kP = 0.01;
    return kP * error;
  }

  public void stop() {
    motor.set(0);
  }
}
