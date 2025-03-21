// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AlgaeIntakeCommand;
import frc.robot.commands.AlgaeShootCommand;
import frc.robot.commands.AlignWithNearest;
import frc.robot.commands.ArmLower;
import frc.robot.commands.ArmRaise;
import frc.robot.commands.ElevatorDown;
import frc.robot.commands.ElevatorMoveLevel3;
import frc.robot.commands.ElevatorPrevPosition;
import frc.robot.commands.ElevatorStop;
import frc.robot.commands.ElevatorUp;
import frc.robot.commands.SlowMode;
import frc.robot.commands.algaeStopIntake;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.FlexAutoSubsystem;
import frc.robot.subsystems.VisionSubsystem;
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
  final CommandGenericHID genericHID = new CommandGenericHID(1);
  // The robot's subsystems and commands are defined here...
  public static final SwerveSubsystem drivebase =
      new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve/neo"));

  private MechanismLigament2d m_elevator;
  private MechanismLigament2d m_wrist;
  private MechanismLigament2d m_wrist2;

  public static PathConstraints Pathconstraints;
  public static FlexAutoSubsystem flexAutoSubsystem;

  // public Command repeatWristDown = new RepeatCommand(new WristDown(coralHoldAngleSubsystem));

  // public Command repeatWristDown = new RepeatCommand(new WristDown(coralHoldAngleSubsystem));

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

  public static ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  // public static ClimberSubsystem climberSubsystem = new ClimberSubsystem();
  // public static CoralHoldSubsystem coralHoldSubsystem = new CoralHoldSubsystem();
  public static VisionSubsystem visionSubsystem = new VisionSubsystem();
  public static AlgaeSubsystem algaeSubsystem = new AlgaeSubsystem();
  // public static CoralHoldAngleSubsystem coralHoldAngleSubsystem = new CoralHoldAngleSubsystem();
  public static ClimberSubsystem climberSubsystem = new ClimberSubsystem();

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
    flexAutoSubsystem = new FlexAutoSubsystem();
    Pathconstraints =
        new PathConstraints(
            drivebase.getMaximumChassisVelocity(),
            4.0,
            drivebase.getMaximumChassisAngularVelocity(),
            Units.degreesToRadians(720));
    // Configure the trigger bindings
    DriverStation.silenceJoystickConnectionWarning(true);
    /*
    new EventTrigger("DropCoral")
        .onTrue(
            new SequentialCommandGroup(
                new ElevatorUp(elevatorSubsystem, 0.5),
                new WaitCommand(0.1),
                new ElevatorStop(elevatorSubsystem),
                repeatWristDown,
                new WaitCommand(0.5),
                new CoralShootCommand(coralHoldSubsystem, this)));
    new EventTrigger("ElevatorLvl1")
        .onTrue(
            new SequentialCommandGroup(
                new ElevatorUp(elevatorSubsystem, 0.65),
                new WaitCommand(0.5),
                new ElevatorStop(elevatorSubsystem)));
    new EventTrigger("GrabCoral")
        .onTrue(
            new SequentialCommandGroup(
                new ElevatorMoveLevel1(elevatorSubsystem),
                new WaitCommand(1),
                new CoralIntakeCommand(coralHoldSubsystem),
                new WaitCommand(2)));
                */
  }

  public void init() {
    configureBindings();

    Mechanism2d mech = new Mechanism2d(3, 3);
    MechanismRoot2d root = mech.getRoot("climber", 2, 0);
    m_elevator =
        root.append(new MechanismLigament2d("elevator", ElevatorSubsystem.levels.length, 90));
    m_wrist =
        m_elevator.append(
            new MechanismLigament2d("Coral", 0.5, 90, 6, new Color8Bit(Color.kPurple)));
    m_wrist2 =
        m_elevator.append(
            new MechanismLigament2d("Algae", 0.25, 90, 3, new Color8Bit(Color.kBlue)));
    SmartDashboard.putData("Mech2d", mech);
  }

  public void Periodic() {
    m_elevator.setLength(elevatorSubsystem.getPositionEncoder());
    // m_wrist.setAngle(coralHoldAngleSubsystem.getEncoder());
    // m_wrist2.setAngle(climberSubsystem.getEncoder());
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

    drivebase.setDefaultCommand(driveRobotOrientedAngularVelocity);
    driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
    driverXbox.x().onTrue(new AlignWithNearest());
    // driverXbox.b().whileTrue(new CoralIntakeCommand(coralHoldSubsystem));
    // driverXbox.b().onFalse(new stopCoralIntake(coralHoldSubsystem));
    driverXbox.y().whileTrue(new AlgaeIntakeCommand(algaeSubsystem));
    // driverXbox.y().onFalse(new algaeStopIntake(algaeSubsystem));

    driverXbox.leftBumper().onTrue(new ElevatorMoveLevel3(elevatorSubsystem));
    driverXbox.rightTrigger().onFalse(new ElevatorStop(elevatorSubsystem));
    driverXbox.rightBumper().onTrue(new ElevatorPrevPosition(elevatorSubsystem));
    driverXbox.leftTrigger().onFalse(new ElevatorStop(elevatorSubsystem));
    driverXbox.leftTrigger().whileTrue(new ElevatorUp(elevatorSubsystem, 0.65));
    driverXbox.rightTrigger().onTrue(new ElevatorDown(elevatorSubsystem, 0.1));
    drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    driverXbox.povUp().whileTrue(new ArmRaise(climberSubsystem));
    driverXbox.povDown().whileTrue(new ArmLower(climberSubsystem));

    driverXbox.povLeft().whileTrue(new AlgaeShootCommand(algaeSubsystem).withTimeout(0.5));
    // driverXbox.povRight().whileTrue(new CoralShootCommand(coralHoldSubsystem).withTimeout(0.5));
    driverXbox.povLeft().onFalse(new algaeStopIntake(algaeSubsystem));
    // driverXbox.povRight().onFalse(new stopCoralIntake(coralHoldSubsystem));
    driverXbox.start().onTrue(new SlowMode());

    // TEMP! Replace with the actual commands once we have the keyboard
    // assistGenericHID.button(0).onTrue(new SequentialCommandGroup(new GoToCommand(1), new
    // AlignWithNearest()));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand(String pathString) {
    // An example command will be run in autonomous
    return drivebase.getAutonomousCommand(pathString);
  }

  public Command getCoralPathCommand(String chooser) {
    Optional<Alliance> ally = DriverStation.getAlliance();
    if (ally.get() == Alliance.Blue) {
      if (chooser.equals("left")) {
        return drivebase.driveToPose(new Pose2d(1.4f, 7f, new Rotation2d(Math.toRadians(65))));
      } else {
        return drivebase.driveToPose(new Pose2d(1.2f, 1f, new Rotation2d(Math.toRadians(-125))));
      }
    } else {
      if (chooser.equals("left")) {
        return drivebase.driveToPose(new Pose2d(16.4f, 1f, new Rotation2d(Math.toRadians(-54))));
      } else {
        return drivebase.driveToPose(new Pose2d(16.3f, 7f, new Rotation2d(Math.toRadians(52))));
      }
    }
  }

  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }
}
