// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.Pathfinding;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.FlexAutoSubsystem;
import java.util.Random;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  private static Robot instance;
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private Timer disabledTimer;

  private String m_autoSelected;
  private final SendableChooser<String> m_AutoChooser = new SendableChooser<>();

  int flexcooldown = 0;

  PigeonIMU mPigeonIMU = new PigeonIMU(0);
  int loopcount = 0;
  private NetworkTableInstance ntInstance;
  private NetworkTable table;
  private NetworkTableEntry poseEntry;
  Random random = new Random();

  public Robot() {
    instance = this;

    // Setup posting Pose data to Networktables for AdvantageScope
    ntInstance = NetworkTableInstance.getDefault();
    table = ntInstance.getTable("AdvantageScope");
    poseEntry = table.getEntry("RobotPose");

    m_AutoChooser.setDefaultOption("Default Auto Close", "Default Drop C");
    // flex auto will find the first coral/or algae it sees, score it on the reef and repeat until
    // disabled
    m_AutoChooser.addOption("Default Auto Middle", "Default Drop M");
    m_AutoChooser.addOption("VDefault Auto Far", "Default Drop F");
    SmartDashboard.putBoolean("FlexAutoEnabled", false);
    SmartDashboard.putData("Auto choices", m_AutoChooser);
  }

  public static Robot getInstance() {
    return instance;
  }

  // Compliments of Co-pilot
  public void updatePose(Pose2d pose) {
    double[] poseArray = new double[] {pose.getX(), pose.getY(), pose.getRotation().getDegrees()};
    poseEntry.setDoubleArray(poseArray);
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

    // Create a timer to disable motor brake a few seconds after disable.  This will let the robot
    // stop
    // immediately when disabled, but then also let it be pushed more
    disabledTimer = new Timer();
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
    // Example pose update
    Pose2d currentPose = new Pose2d(1.0, 2.0, new Rotation2d(Math.toRadians(45)));
    updatePose(currentPose);
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    m_robotContainer.setMotorBrake(true);
    if (disabledTimer == null) {
      disabledTimer = new Timer();
    }
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
    m_robotContainer.scheduleSelectedSubsystem();
    m_autoSelected = m_AutoChooser.getSelected();
    System.out.println("Auto selected: " + m_autoSelected);

    if (SmartDashboard.getBoolean("FlexAutoEnabled", false)) {
      Pathfinding.setPathfinder(new FlexAutoSubsystem());
    }
    m_autonomousCommand = m_robotContainer.getAutonomousCommand(m_autoSelected);

    m_robotContainer.setMotorBrake(true);

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    // if pathfinding can get a new make one and run it
    if (Pathfinding.isNewPathAvailable() && flexcooldown >= 500) {
      PathPlannerPath path = Pathfinding.getCurrentPath(null, null);
      m_autonomousCommand.andThen(AutoBuilder.followPath(path));
      flexcooldown = 0;
    } else if (flexcooldown < 500) {
      flexcooldown += 1;
    }
  }

  @Override
  public void teleopInit() {
    m_robotContainer.scheduleSelectedSubsystem();
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    } else {
      CommandScheduler.getInstance().cancelAll();
    }
    m_robotContainer.setDriveMode();
    m_robotContainer.setMotorBrake(true);

    if (loopcount > 10) {
      loopcount = 0;
      double[] ypr = new double[3];
      mPigeonIMU.getYawPitchRoll(ypr);
      DataLogManager.log("IMU: " + ypr);
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
    m_robotContainer.scheduleSelectedSubsystem();
    m_robotContainer.setDriveMode();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
    m_robotContainer.simulationInit();
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
    m_robotContainer.simulationPeriodic();
  }
}
