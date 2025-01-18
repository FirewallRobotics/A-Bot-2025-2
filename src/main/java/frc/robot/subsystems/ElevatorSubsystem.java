package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
  private final SparkFlex motor;
  private final Encoder encoder;

  // Elevator levels in encoder ticks
  private final int[] levels = {0, 1000, 2000, 3000, 4000};

  public ElevatorSubsystem() {
    motor = new SparkFlex(0, MotorType.kBrushless); // Assign motor controller port
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

  public boolean isFinished(int position){
    if(levels[position] - encoder.get() == 0){
      return true;
    }else{
      return false;
    }
  }
}
