# Robot Architecture Overview

## Drive System
1. **YAGSL (Yet Another Generic Swerve Library)**
- Current implementation uses YAGSL for swerve drive control
- Key components:
  ```java
  SwerveSubsystem - Primary drive control
  SwerveInputStream - Processes driver inputs
  ```
- Located in RobotContainer.java:
  ```java
  public static final SwerveSubsystem drivebase = new SwerveSubsystem(
    new File(Filesystem.getDeployDirectory(), "swerve/neo")
  );
  ```

2. **Legacy Code Note**
- Files in /commands/swervedrive/* are from 2024 drive system
- These files used different hardware configuration
- Current robot movement uses YAGSL methods:
  ```java
  drivebase.drive(xSpeed, ySpeed, rotation, fieldRelative, openLoop)
  drivebase.driveFieldOriented(inputStream)
  drivebase.centerModulesCommand()
  ```

## Core Framework
1. **TimedRobot Framework**
-

Robot

 extends

TimedRobot

 which runs periodic functions every 20ms
- Key periodic methods:
  ```java
  robotPeriodic() // Always runs
  autonomousPeriodic() // During auto
  teleopPeriodic() // During teleop
  testPeriodic() // During test mode
  ```

## Vision System
2. **Limelight Integration**
-

LimelightHelpers

 provides vision processing
- Key features:
  ```java
  getTV() // Check valid target
  getBotPose3d() // Get robot pose
  setPipelineIndex() // Switch vision pipelines
  ```
- Data flows through NetworkTables to Driver Station

## Path Planning
3. **PathPlanner**
- Located in

pathplanner


- Autonomous path following:
  ```java
  // In Robot.java
  autonomousPeriodic() {
    flexAutoSubsystem.CreatePath(constraints, m_CoralStationChooser.getSelected());
  }
  ```

## Visualization
4. **AdvantageScope**
- Receives data through NetworkTables for visualization
- Key data points:
  - Robot pose
  - Vision targets
  - Autonomous paths
  - Subsystem states

## Control Systems
5. **Driver Station Integration**
- Sends data through SmartDashboard:
  ```java
  // In Robot.java
  SmartDashboard.putData(m_chooser); // Auto selector
  SmartDashboard.putBoolean("AutoDone", autonomousCommand.isFinished());
  ```
- Receives operator input through:
  ```java
  DriverStation.isDisabled()
  CommandScheduler.getInstance().run()
  ```

## Data Flow
The data flow is:
```
Sensors (Limelight, Encoders) -> NetworkTables -> Driver Station/AdvantageScope
Driver Input -> Command Scheduler -> Subsystems -> Robot Actions
```

This architecture follows standard FRC practices using WPILib's command-based framework.
