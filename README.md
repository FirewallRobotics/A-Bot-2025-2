# 2025-A-Bot

# Team 5607 Robot Code (2025)

## Hardware Configuration
- Drive Train: Swerve Drive
- Motor Controllers:
  - Swerve Drive: REV NEO motors with SPARKS MAX
  - Swerve Rotation: REV NEO 550 motors with SPARKS MAX
- Sensors:
  - NavX-MXP Gyroscope
  - CANCoder for absolute position

## Required Vendor Dependencies
- REV Robotics REVLib
- Phoenix Framework (for CANCoders)
- NavX Libraries
- PathPlanner Library
- LimelightLib

## Vision System
- Limelight 3 Camera
  - Location: Front of robot
  - Primary use: AprilTag detection
  - Network table: `limelight`
  - Configuration at http://limelight.local:5801

## Swerve Configuration
Located in `src/main/deploy/swerve/`:
- `swerve.json`: Main swerve drive configuration
- `modules/`: Individual module configurations
  - `frontleft.json`
  - `frontright.json`
  - `backleft.json`
  - `backright.json`

## PathPlanner Configuration
Located in `src/main/deploy/pathplanner/`:
- `paths/`: Individual path configurations
- `autos/`: Autonomous routines
- Path constraints:
  - Max velocity: 4.5 m/s
  - Max acceleration: 3.0 m/s²
  - Holonomic rotation constraints enabled

## Vision Processing
VisionSubsystem features:
- AprilTag pose estimation
- Robot field position updates
- Vision-based autonomous alignment
- NetworkTables integration for dashboard feedback

## Development Setup
1. Install WPILib 2025
2. Install vendor dependencies using WPILib VS Code
3. Configure team number (5607) in `.wpilib/wpilib_preferences.json`
4. Verify swerve configurations match physical robot setup

## Building and Deployment
```bash
./gradlew build  # Build the project
./gradlew deploy # Deploy to robot
```

## Code Structure
- `src/main/java/frc/robot/`
  - `Constants.java`: Robot-wide constants
  - `subsystems/`: Robot subsystems
  - `commands/`: Robot commands

## Documentation Organization
All documentation files should be placed in the `./doc` folder with clear naming:

1. Test-related docs: `TESTING.md`
2. Vision system docs: `VISION_CHANGES_FOR_TESTING.md`
3. Architecture docs: `ARCHITECTURE.md`

## Best Practices
- Use consistent file paths in documentation
- Keep docs close to code
- Use relative links between docs
- Maintain a clear structure

## Note to Copilot
When creating documentation:
1. Always use `/doc` folder
2. Use consistent filepath comments
3. Follow existing naming patterns

[![CI](https://github.com/FirewallRobotics/A-Bot-2025-2/actions/workflows/main.yml/badge.svg)](https://github.com/FirewallRobotics/A-Bot-2025-2/actions/workflows/main.yml)

2025 Robot (A-Bot) code
