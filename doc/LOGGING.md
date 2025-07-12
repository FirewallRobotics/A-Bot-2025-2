# Robot Logging Best Practices

## Overview
This guide explains how to implement logging across Commands and Subsystems using:
- WPILib DataLog
- NetworkTables
- AdvantageScope visualization

## Setup

### 1. Create DataLog Instance
```java
// filepath: src/main/java/frc/robot/Robot.java
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;

public class Robot extends TimedRobot {
    @Override
    public void robotInit() {
        // Start data logging
        DataLogManager.start();
        // Record both DS control and joystick data
        DriverStation.startDataLog(DataLogManager.getLog());
    }
}
```

### 2. Subsystem Logging
```java
// filepath: src/main/java/frc/robot/subsystems/ExampleSubsystem.java
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.util.datalog.DoubleLogEntry;

public class ExampleSubsystem extends SubsystemBase {
    // Log entries
    private final DoubleLogEntry speedLog;
    private final DoubleLogEntry positionLog;

    public ExampleSubsystem() {
        speedLog = new DoubleLogEntry(DataLogManager.getLog(), "/example/speed");
        positionLog = new DoubleLogEntry(DataLogManager.getLog(), "/example/position");
    }

    @Override
    public periodic() {
        // Log to DataLog (stored on RIO)
        speedLog.append(getCurrentSpeed());
        positionLog.append(getCurrentPosition());

        // Log to NetworkTables (real-time display)
        SmartDashboard.putNumber("Example/Speed", getCurrentSpeed());
        SmartDashboard.putNumber("Example/Position", getCurrentPosition());
    }
}
```

### 3. Command Logging
```java
// filepath: src/main/java/frc/robot/commands/ExampleCommand.java
public class ExampleCommand extends CommandBase {
    @Override
    public void initialize() {
        DataLogManager.log(String.format("Starting %s command", getName()));
    }

    @Override
    public void execute() {
        // Log important state changes
        if (significantEventOccurred) {
            DataLogManager.log("Significant event in " + getName());
        }
    }

    @Override
    public void end(boolean interrupted) {
        DataLogManager.log(String.format("%s command ended. Interrupted: %b", getName(), interrupted));
    }
}
```

## Logging Categories

### Critical Data (Always Log)
- Robot pose
- Subsystem states
- Command execution
- Error conditions
- Game piece states
- Auto path selection

### Diagnostic Data (Development/Testing)
- Motor currents
- Voltage levels
- Temperature readings
- Network latency
- Vision processing stats

## AdvantageScope Integration

### 1. Create Logging Layout
```java
// filepath: src/main/java/frc/robot/utils/LoggingUtil.java
public class LoggingUtil {
    public static void logMechanismState(String mechanism, String state) {
        // Log to both DataLog and NetworkTables for AdvantageScope
        DataLogManager.log(String.format("%s: %s", mechanism, state));
        SmartDashboard.putString("Mechanisms/" + mechanism, state);
    }
}
```

### 2. Field Visualization
```java
// filepath: src/main/java/frc/robot/subsystems/DriveSubsystem.java
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

public class DriveSubsystem extends SubsystemBase {
    private final Field2d field = new Field2d();

    public DriveSubsystem() {
        SmartDashboard.putData("Field", field);
    }

    @Override
    public periodic() {
        // Update robot pose on field
        field.setRobotPose(getPose());
    }
}
```

## Best Practices

1. Use hierarchical naming for log entries:
   - `/subsystem/component/measurement`
   - `/commands/commandName/state`

2. Log frequency guidelines:
   - High-frequency data (position, velocity): Every loop (20ms)
   - Status changes: On event
   - Diagnostic data: 100-500ms intervals

3. Include timestamps with logged events

4. Use consistent units (meters, radians, seconds)

5. Create replay-friendly logs:
   - Log all inputs needed to recreate behavior
   - Include field position and robot state
   - Log vision targets and game piece locations

## File Management

Logs are stored on the RIO at:
- `/home/lvuser/logs/`

Configure log rotation to prevent storage issues:
```java
// filepath: src/main/java/frc/robot/Robot.java
DataLogManager.logGenerator("Robot_" + timestamp);
```

## Viewing Logs

1. Download logs using:
   - WPILib Data Log Tool
   - SSH/SCP from RIO
   - USB drive

2. Open in AdvantageScope:
   - Import .wpilog file
   - Configure visualizations
   - Play back match data
```

This guide provides a comprehensive logging strategy that will help with:
- Real-time debugging via Driver Station
- Post-match analysis
- Simulation validation
- Development troubleshooting

Would you like me to expand on any particular section?
This guide provides a comprehensive logging strategy that will help with:
- Real-time debugging via Driver Station
- Post-match analysis
- Simulation validation
- Development troubleshooting

Would you like me to expand on any particular section?
