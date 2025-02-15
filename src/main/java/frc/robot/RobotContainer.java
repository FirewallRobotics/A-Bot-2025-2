// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.events.EventTrigger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ArmLower;
import frc.robot.commands.ArmRaise;
import frc.robot.commands.CoralIntakeCommand;
import frc.robot.commands.CoralShootCommand;
import frc.robot.commands.ElevatorMoveLevel1;
import frc.robot.commands.ElevatorNextPosition;
import frc.robot.commands.ElevatorPrevPosition;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CoralHoldSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import java.util.Optional;
import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
@SuppressWarnings("unused")
public class RobotContainer {

  // Replace with CommandPS4Controller or CommandJoystick if needed
  final CommandXboxController driverXbox = new CommandXboxController(0);
  // The robot's subsystems and commands are defined here...
  public static final SwerveSubsystem drivebase =
      new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve/neo"));

  private MechanismLigament2d m_elevator;
  private MechanismLigament2d m_wrist;

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular
   * velocity.
   */
  SwerveInputStream driveAngularVelocity =
      SwerveInputStream.of(
              drivebase.getSwerveDrive(),
              () -> driverXbox.getLeftY() * -1,
              () -> driverXbox.getLeftX() * -1)
          .withControllerRotationAxis(driverXbox::getRightX)
          .deadband(OperatorConstants.DEADBAND)
          .scaleTranslation(0.8)
          .allianceRelativeControl(true);

  /** Clone's the angular velocity input stream and converts it to a fieldRelative input stream. */
  SwerveInputStream driveDirectAngle =
      driveAngularVelocity
          .copy()
          .withControllerHeadingAxis(driverXbox::getRightX, driverXbox::getRightY)
          .headingWhile(true);

  private ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  private ClimberSubsystem climberSubsystem = new ClimberSubsystem();
  private CoralHoldSubsystem coralHoldSubsystem = new CoralHoldSubsystem();

  /** Clone's the angular velocity input stream and converts it to a robotRelative input stream. */
  SwerveInputStream driveRobotOriented =
      driveAngularVelocity.copy().robotRelative(true).allianceRelativeControl(false);

  SwerveInputStream driveAngularVelocityKeyboard =
      SwerveInputStream.of(
              drivebase.getSwerveDrive(),
              () -> -driverXbox.getLeftY(),
              () -> -driverXbox.getLeftX())
          .withControllerRotationAxis(() -> driverXbox.getRawAxis(2))
          .deadband(OperatorConstants.DEADBAND)
          .scaleTranslation(0.8)
          .allianceRelativeControl(true);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleKeyboard =
      driveAngularVelocityKeyboard
          .copy()
          .withControllerHeadingAxis(
              () -> Math.sin(driverXbox.getRawAxis(2) * Math.PI) * (Math.PI * 2),
              () -> Math.cos(driverXbox.getRawAxis(2) * Math.PI) * (Math.PI * 2))
          .headingWhile(true);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    DriverStation.silenceJoystickConnectionWarning(true);
    elevatorSubsystem = new ElevatorSubsystem();

    new EventTrigger("DropCoral").onTrue(new CoralShootCommand(coralHoldSubsystem));
    new EventTrigger("ElevatorLvl1").onTrue(new ElevatorMoveLevel1(elevatorSubsystem));
    new EventTrigger("GrabCoral")
        .onTrue(new CoralIntakeCommand(coralHoldSubsystem, elevatorSubsystem));
  }

  public void init() {
    configureBindings();

    Mechanism2d mech = new Mechanism2d(3, 3);
    MechanismRoot2d root = mech.getRoot("climber", 2, 0);
    m_elevator =
        root.append(new MechanismLigament2d("elevator", ElevatorSubsystem.levels.length, 90));
    m_wrist =
        m_elevator.append(
            new MechanismLigament2d("wrist", 0.5, 90, 6, new Color8Bit(Color.kPurple)));
    SmartDashboard.putData("Mech2d", mech);
  }

  public void Periodic() {
    m_elevator.setLength(0.25 + (SmartDashboard.getNumber("ElevatorPos", 0) / 3));
    m_wrist.setAngle(90);
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

    Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveRobotOrientedAngularVelocity = drivebase.driveFieldOriented(driveRobotOriented);
    Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngle);
    Command driveFieldOrientedDirectAngleKeyboard =
        drivebase.driveFieldOriented(driveDirectAngleKeyboard);
    Command driveFieldOrientedAnglularVelocityKeyboard =
        drivebase.driveFieldOriented(driveAngularVelocityKeyboard);
    Command driveSetpointGenKeyboard =
        drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngleKeyboard);

    if (RobotBase.isSimulation()) {
      drivebase.setDefaultCommand(driveRobotOrientedAngularVelocity);
    } else {
      drivebase.setDefaultCommand(driveRobotOrientedAngularVelocity);
    }
    driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
    driverXbox.x().onTrue(Commands.none());

    Optional<Alliance> ally = DriverStation.getAlliance();
    if (ally.isPresent()) {
      if (ally.get() == Alliance.Blue) {
        driverXbox
            .b()
            .whileTrue(
                drivebase.driveToPose(new Pose2d(new Translation2d(4, 4), drivebase.getHeading())));
      }
      if (ally.get() == Alliance.Red) {
        driverXbox
            .b()
            .whileTrue(
                drivebase.driveToPose(
                    new Pose2d(new Translation2d(13, 4), drivebase.getHeading())));
      }
    }
    driverXbox.start().whileTrue(Commands.none());
    driverXbox.back().whileTrue(Commands.none());
    driverXbox.leftBumper().onTrue(new ElevatorNextPosition(elevatorSubsystem));
    driverXbox.rightBumper().onTrue(new ElevatorPrevPosition(elevatorSubsystem));
    drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    driverXbox.y().whileTrue((new ArmLower(climberSubsystem)));
    driverXbox.x().whileTrue((new ArmRaise(climberSubsystem)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return drivebase.getAutonomousCommand("New Auto");
  }

  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }
}
