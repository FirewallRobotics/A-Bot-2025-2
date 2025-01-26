// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.CanBusLogger;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // Replace with CommandPS4Controller or CommandJoystick if needed
  final CommandXboxController driverXbox = new CommandXboxController(0);
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase =
      new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve/neo"));
  private final CanBusLogger canBusLogger = new CanBusLogger(); // Example device ID

  // SendableChooser for SmartDashboard
  private final SendableChooser<SubsystemBase> chooser = new SendableChooser<>();

  // Applies deadbands and inverts controls because joysticks
  // are back-right positive while robot
  // controls are front-left positive
  // left stick controls translation
  // right stick controls the angular velocity of the robot
  Command driveFieldOrientedAnglularVelocity =
      drivebase.driveCommand(
          () ->
              MathUtil.applyDeadband(driverXbox.getLeftY() * -1, OperatorConstants.LEFT_Y_DEADBAND),
          () ->
              MathUtil.applyDeadband(driverXbox.getLeftX() * -1, OperatorConstants.LEFT_X_DEADBAND),
          () -> driverXbox.getRightX());

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // create all the smartdashboard values
    SmartDashboard.putNumber("AutoScanSpeed", 1.0);
    SmartDashboard.putNumber("AutoRotateSpeed", 1.0);
    SmartDashboard.putNumber("AutoMoveSpeed", 1.0);
    SmartDashboard.putNumber("AssistLossRange", 1.0);
    SmartDashboard.putNumber("AssistProcessorDistance", 1.0);
    SmartDashboard.putNumber("AssistReefDistance", 1.0);
    // Configure the trigger bindings
    configureBindings();
    // Setup DriverStation control for turning on CanBus Diags
    // Use canBusLogger to log some information
    SmartDashboard.putString("CanBusLogger", "RobotContainer initialized CanBusLogger");
    // Add options to the chooser
    chooser.setDefaultOption("None", null);
    chooser.addOption("CanBusLogger", canBusLogger);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    if (Robot.isSimulation()) {
      driverXbox
          .start()
          .onTrue(
              Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
    }
    if (DriverStation.isTest()) {
      driverXbox.b().whileTrue(drivebase.sysIdDriveMotorCommand());
      // driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      // driverXbox.y().whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
      driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverXbox.back().whileTrue(drivebase.centerModulesCommand());
      driverXbox.leftBumper().onTrue(Commands.none());
      driverXbox.rightBumper().onTrue(Commands.none());
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    } else {
      driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      // driverXbox.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
      driverXbox
          .b()
          .whileTrue(
              drivebase.driveToPose(
                  new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0))));
      // driverXbox.y().whileTrue(drivebase.aimAtSpeaker(2));
      driverXbox.start().whileTrue(Commands.none());
      driverXbox.back().whileTrue(Commands.none());
      driverXbox.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverXbox.rightBumper().onTrue(Commands.none());
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand(String pathName) {
    // An example command will be run in autonomous
    return drivebase.getAutonomousCommand(pathName);
  }

  /**
   * Schedules the selected subsystem from the chooser.
   * If a subsystem is selected, it registers the subsystem with the CommandScheduler.
   * Initial use is to schedule the CanBusLogger subsystem when selected on the SmartDashboard.
   */
  public void scheduleSelectedSubsystem() {
    SubsystemBase selectedSubsystem = chooser.getSelected();
    if (selectedSubsystem != null) {
      CommandScheduler.getInstance().registerSubsystem(selectedSubsystem);
    }
  }

  public void setDriveMode() {
    configureBindings();
  }

  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }
}
