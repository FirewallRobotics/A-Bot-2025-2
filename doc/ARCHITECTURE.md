Collecting workspace information

Here's how the architecture works in this codebase:

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

4. **AdvantageScope**
- Receives data through NetworkTables for visualization
- Key data points:
  - Robot pose
  - Vision targets
  - Autonomous paths
  - Subsystem states

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

The data flow is:
```
Sensors (Limelight, Encoders) -> NetworkTables -> Driver Station/AdvantageScope
Driver Input -> Command Scheduler -> Subsystems -> Robot Actions
```

This architecture follows standard FRC practices using WPILib's command-based framework.