package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class UltrasonicSensor extends SubsystemBase {

  // Create an Ultrasonic object with the appropriate ports for the RoboRIO
  private Ultrasonic ultrasonicSensorCoral;
  private Ultrasonic ultrasonicSensorAlgae;

  public UltrasonicSensor() {
    // Initialize the Ultrasonic sensor on the given input/output ports
    ultrasonicSensorCoral = new Ultrasonic(1, 0); // Ports 1 and 0 (example ports)
    ultrasonicSensorAlgae = new Ultrasonic(3, 2); // Ports 2 and 3 (example ports)

    // Enable the ultrasonic sensor
    Ultrasonic.setAutomaticMode(true);
  }

  /**
   * Get the distance from the Coral ultrasonics sensor
   *
   * @return The distance between the Coral ultrasonics sensor and the nearest object
   */
  public double getDistanceCoral() {
    // Get the distance in inches
    double distanceInches = ultrasonicSensorCoral.getRangeInches();

    // Output the distance to the SmartDashboard
    SmartDashboard.putNumber("Ultrasonic Coral Distance", distanceInches);

    return distanceInches;
  }

  /**
   * Get the distance from the Algae ultrasonics sensor
   *
   * @return The distance between the Algae ultrasonics sensor and the nearest object
   */
  public double getDistanceAlgae() {
    // Get the distance in inches
    double distanceInches = ultrasonicSensorAlgae.getRangeInches();

    // Output the distance to the SmartDashboard
    SmartDashboard.putNumber("Ultrasonic Algae Distance", distanceInches);

    return distanceInches;
  }
}
