# SearchForTag Command

Create SearchForTag.java class that extends Command to search for AprilTags using the Limelight vision system.

## Requirements

1. Use SwerveSubsystem from YAGSL (Yet Another Generic Swerve Library) as the drive system
2. Check for AprilTags in current Limelight camera view by searching for latest entries in the NetworkTables
3. If no entries are found within 30 milliseconds:
   - Use SwerveSubsystem.drive(xSpeed, ySpeed, rotationSpeed, fieldRelative, openLoop) method
   - Set rotation speed to 0.3 for clockwise spin
   - Keep xSpeed and ySpeed at 0
   - Use fieldRelative = true
4. Stop rotation when an AprilTag is detected in NetworkTables
5. Place command class in: src/main/java/frc/robot/commands/
6. Follow logging best practices from doc/LOGGING.md
7. Create unit tests following doc/Testing.md guidelines
   - Place tests in: src/test/java/frc/robot/commands/

## Example Usage

The command should be used with the YAGSL SwerveSubsystem:

```java
// Example command creation
SearchForTag searchCommand = new SearchForTag(RobotContainer.drivebase);

// Example binding to controller button
driverController.x().whileTrue(searchCommand);
```
