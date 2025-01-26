package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.hal.CANData;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.List;

/**
 * The CanBusLogger class is responsible for logging CAN bus data and alerts.
 * It extends the SubsystemBase class and periodically logs CAN signal data
 * from devices on the CAN bus. It also checks for errors and alerts related
 * to the CAN bus and posts them to NetworkTables. It's purpose is to help diagnose 
 * problems on the CAN bus and monitor the health of devices connected to it.
 * 
 * <p>Key functionalities include:
 * <ul>
 *   <li>Polling the CAN bus for devices and maintaining a list of device IDs.</li>
 *   <li>Logging CAN signal data periodically.</li>
 *   <li>Checking for errors and alerts on the CAN bus and posting them to NetworkTables.</li>
 * </ul>
 * 
 * <p>Usage:
 * <pre>
 * {@code
 * CanBusLogger canBusLogger = new CanBusLogger();
 * canBusLogger.periodic();
 * }
 * </pre>
 * 
 * <p>Methods:
 * <ul>
 *   <li>{@link #CanBusLogger()}: Constructs a new CanBusLogger instance.</li>
 *   <li>{@link #getTimer()}: Returns the timer used for logging intervals.</li>
 *   <li>{@link #periodic()}: Logs CAN signal data periodically.</li>
 *   <li>{@link #pollCanBusForDevices()}: Polls the CAN bus for devices and updates the device list.</li>
 *   <li>{@link #getDeviceIds()}: Returns the list of device IDs.</li>
 *   <li>{@link #setDeviceIds(List)}: Sets the list of device IDs.</li>
 *   <li>{@link #logCanSignals()}: Logs CAN signals from devices on the CAN bus.</li>
 *   <li>{@link #bytesToHex(byte[])}: Converts a byte array to a hexadecimal string.</li>
 *   <li>{@link #checkForErrors()}: Checks for errors and alerts on the CAN bus.</li>
 * </ul>
 * 
 * <p>Example:
 * <pre>
 * {@code
 * CanBusLogger logger = new CanBusLogger();
 * logger.pollCanBusForDevices();
 * logger.logCanSignals();
 * }
 * </pre>
 * 
 * <p>Note: This class assumes that device IDs range from 0 to 63 and uses a specific CAN signal ID (0x1FF) for logging.
 * 
 * @author 
 * @version 1.0
 * @since 2023-10-01
 */
public class CanBusLogger extends SubsystemBase {
  private static final double LOG_INTERVAL_SECONDS = 1.0;
  private final Timer timer;
  private final NetworkTableInstance ntInstance;
  private final NetworkTable table;
  private final NetworkTableEntry canDataEntry;
  private final NetworkTableEntry canBusAlertEntry;
  private final List<Integer> deviceIds;
  private int currentDeviceIndex;

  /**
   * Constructs a new CanBusLogger instance.
   * Initializes the timer and starts it.
   * Sets up the NetworkTable instance and entries for logging CAN bus data and alerts.
   * Initializes the list of device IDs and polls the CAN bus for devices.
   * Sets the current device index to 0.
   */
  public CanBusLogger() {
    timer = new Timer();
    timer.start();

    ntInstance = NetworkTableInstance.getDefault();
    table = ntInstance.getTable("CanBusLogger");
    canDataEntry = table.getEntry("CanSignalData");
    canBusAlertEntry = table.getEntry("CanBusAlert");

    deviceIds = new ArrayList<>();
    pollCanBusForDevices();
    currentDeviceIndex = 0;
  }

 

  public Timer getTimer() {
    return timer;
  }

  @Override
  public void periodic() {
    // Log CAN signal data periodically
    if (timer.hasElapsed(LOG_INTERVAL_SECONDS)) { // Log every second
      logCanSignals();
      timer.reset();
    }
  }

  public void pollCanBusForDevices() {
    // Example: Poll a range of device IDs to see if they respond
    for (int deviceId = 0; deviceId <= 63; deviceId++) { // Assuming device IDs range from 0 to 63
      CAN canDevice = new CAN(deviceId); // Create a new CAN object for each device ID
      CANData canData = new CANData();
      boolean success = canDevice.readPacketNew(0x1FF, canData);
      if (success) {
        deviceIds.add(deviceId);
      }
      canDevice.close(); // Close CAN object after each iteration
    }

    if (deviceIds.isEmpty()) {
      // Post a CANBus alert to the NetworkTables
      canBusAlertEntry.setString("Device list is empty.");
    } else {
      canBusAlertEntry.setString("Devices found: " + deviceIds.size());
    }
  }

  public List<Integer> getDeviceIds() {
    return deviceIds;
  }

  public void setDeviceIds(List<Integer> deviceIds) {
    this.deviceIds.clear();
    this.deviceIds.addAll(deviceIds);
  }

  /**
   * Logs CAN signals from devices on the CAN bus.
   * 
   * This method iterates through a list of CAN device IDs, reads a specific CAN signal
   * from each device, and logs the data. If an error occurs while reading the CAN data,
   * an error message is logged. The method cycles through the devices in a round-robin
   * fashion.
   * 
   * The method performs the following steps:
   * 1. Checks if the list of device IDs is empty. If it is, the method returns immediately.
   * 2. Retrieves the current device ID from the list and creates a CAN device instance.
   * 3. Attempts to read a specific CAN signal (with ID 0x1FF) from the device.
   * 4. If the read is successful, converts the data to a hexadecimal string and logs it.
   * 5. If an error occurs, logs an error message with the device ID and the exception message.
   * 6. Cycles to the next device in the list.
   * 7. Closes the CAN device instance.
   * 8. Checks for any errors.
   * 9. Closes the CAN device instance again.
   */
  public void logCanSignals() {
    if (deviceIds.isEmpty()) {
      return;
    }

    int deviceId = deviceIds.get(currentDeviceIndex);
    CAN canDevice = new CAN(deviceId);

    try {
      // Example: Read and log a specific CAN signal
      CANData canData = new CANData();
      boolean success = canDevice.readPacketNew(0x1FF, canData);
      if (success) {
        String hexData = bytesToHex(canData.data);
        canDataEntry.setString("Device " + deviceId + ": " + hexData);
      }
    } catch (Exception e) {
      canBusAlertEntry.setString(
          "Error reading CAN data for device " + deviceId + ": " + e.getMessage());
    } finally {
      // Cycle to the next device
      currentDeviceIndex = (currentDeviceIndex + 1) % deviceIds.size();
      canDevice.close();
      checkForErrors();
      canDevice.close();
    }
  }

  public String bytesToHex(byte[] bytes) {
    StringBuilder sb = new StringBuilder(bytes.length * 3);
    for (byte b : bytes) {
      sb.append(String.format("%02X ", b));
    }
    return sb.toString().trim();
  }

  public void checkForErrors() {
    if (deviceIds.isEmpty()) {
      return;
    }

    for (int deviceId : deviceIds) {
      SparkFlex controller = new SparkFlex(deviceId, MotorType.kBrushless);

      // Check for bus off condition
      // The threshold value of 1.0 volts is used because a healthy CAN bus voltage should be
      // higher.
      // If the voltage is below this threshold, it indicates a potential bus off condition.
      if (controller.getBusVoltage() < 1.0) {
        canBusAlertEntry.setString("CAN bus is off for device " + deviceId);
      }

      // Example: Check for error frames

      if (controller.hasActiveFault()) {
        canBusAlertEntry.setString(
            "CAN bus fault detected for device "
                + deviceId
                + ": "
                + controller.getFaults().toString());
      }

      // Example: Check for other errors or warnings
      if (controller.hasActiveWarning()) {
        canBusAlertEntry.setString("Warnings detected for device " + deviceId);
      }
      canBusAlertEntry.setString(
          "Motor temperature for device " + deviceId + " is " + controller.getMotorTemperature());
      canBusAlertEntry.setString(
          "Motor temperature is - " + controller.getMotorTemperature() + " for device " + deviceId);
      controller.close();
    }
  }
}
