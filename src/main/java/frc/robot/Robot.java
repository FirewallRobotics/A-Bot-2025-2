// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.AlignWithNearest;
import frc.robot.subsystems.FlexAutoSubsystem;
import frc.robot.subsystems.UltrasonicSensor;
import java.util.List;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  private static Robot instance;
  public static Command autonomousCommand;

  private RobotContainer m_robotContainer;
  public static UltrasonicSensor globalUltraSensors;
  public static FlexAutoSubsystem flexAutoSubsystem;

  private Timer disabledTimer;

  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  public Robot() {
    SmartDashboard.putBoolean("FlexAuto", false);
    SmartDashboard.putBoolean("AutoThenGoToCoralStation", false);
    SmartDashboard.putNumber("AssistMinDistance", 40);
    SmartDashboard.putNumber("AutoMoveSpeed", 5);
    SmartDashboard.putNumber("AutoScanSpeed", 5);
    instance = this;
    m_chooser.setDefaultOption("Default Drop C", "Default Drop C");
    m_chooser.addOption("Default Drop M", "Default Drop M");
    m_chooser.addOption("Default Drop F", "Default Drop F");
    m_chooser.addOption("FWD 10 feet", "FWD10");
    m_chooser.addOption("FWD 5 feet", "FWD5");
    SmartDashboard.putData(m_chooser);
  }

  public static Robot getInstance() {
    return instance;
  }

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    flexAutoSubsystem = new FlexAutoSubsystem();

    // Create a timer to disable motor brake a few seconds after disable.  This will let the robot
    // stop
    // immediately when disabled, but then also let it be pushed more
    disabledTimer = new Timer();

    if (isSimulation()) {
      DriverStation.silenceJoystickConnectionWarning(true);
    }
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    if (!DriverStation.isDisabled()) {
      m_robotContainer.Periodic();
    }

    // to set the levels
    // SmartDashboard.putNumber("ElevEncoder:",
    // RobotContainer.elevatorSubsystem.getPositionEncoder());
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    m_robotContainer.setMotorBrake(true);
    disabledTimer.reset();
    disabledTimer.start();
  }

  @Override
  public void disabledPeriodic() {
    if (disabledTimer.hasElapsed(Constants.DrivebaseConstants.WHEEL_LOCK_TIME)) {
      m_robotContainer.setMotorBrake(false);
      disabledTimer.stop();
    }
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    RobotContainer.drivebase.zeroGyro();
    m_robotContainer.init();
    m_autoSelected = m_chooser.getSelected();
    System.out.println("Auto selected: " + m_autoSelected);
    autonomousCommand = m_robotContainer.getAutonomousCommand(m_autoSelected);

    // if (m_autoSelected.contains("Drop")) {
    //  autonomousCommand.andThen(
    //      new CoralShootCommand(RobotContainer.coralHoldSubsystem), new WaitCommand(1));
    // }

    // schedule the autonomous command
    if (SmartDashboard.getBoolean("AutoThenGoToCoralStation", false)) {
      autonomousCommand.andThen(m_robotContainer.getCoralPathCommand());
      if (SmartDashboard.getBoolean("FlexAuto", false)) {
        autonomousCommand.andThen(new AlignWithNearest());
        // new CoralIntakeCommand(RobotContainer.coralHoldSubsystem),
        // new WaitCommand(1));
      }
    }
    autonomousCommand.schedule();
  }

  List<Pose2d> points;

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    // if flex auto enabled and we are not moving (flex checks this using .isnewpathavailable() )
    if (SmartDashboard.getBoolean("FlexAuto", false)
        && flexAutoSubsystem.isNewPathAvailable()
        && autonomousCommand.isFinished()) {
      // create robots constraints
      PathConstraints constraints =
          new PathConstraints(
              RobotContainer.drivebase.getMaximumChassisVelocity(),
              4.0,
              RobotContainer.drivebase.getMaximumChassisAngularVelocity(),
              Units.degreesToRadians(720));

      // have flex create points to follow
      flexAutoSubsystem.CreatePath(constraints);
    }
    if (autonomousCommand.isFinished()) {
      autonomousCommand = Commands.none();
    }
  }

  @Override
  public void teleopInit() {
    // RobotContainer.drivebase.zeroGyro();
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    m_robotContainer.init();
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
      RobotContainer.drivebase.drive(new Translation2d(0, 0), 0, true);
    } else {
      CommandScheduler.getInstance().cancelAll();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    m_robotContainer.init();
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
