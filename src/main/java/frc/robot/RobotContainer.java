// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ElevatorSubsystemConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AlgaeIntakeCommand;
import frc.robot.commands.AlgaeShootCommand;
import frc.robot.commands.AlignWithNearest;
import frc.robot.commands.ArmSetToCoralAccept;
import frc.robot.commands.ArmSetToMiddle;
import frc.robot.commands.ArmSetToScore;
import frc.robot.commands.CoralIntakeCommand;
import frc.robot.commands.CoralShootCommand;
import frc.robot.commands.ElevatorDown;
import frc.robot.commands.ElevatorMoveLevel2;
import frc.robot.commands.ElevatorMoveLevel3;
import frc.robot.commands.ElevatorStop;
import frc.robot.commands.ElevatorUp;
import frc.robot.commands.GoToCommand;
import frc.robot.commands.SlowMode;
import frc.robot.commands.WristDown;
import frc.robot.commands.WristStop;
import frc.robot.commands.WristUp;
import frc.robot.commands.algaeStopIntake;
import frc.robot.commands.stopCoralIntake;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.CoralHoldSubsystem;
import frc.robot.subsystems.CoralWristSubsystem;
import frc.robot.subsystems.ElevatorCoralSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import java.util.Optional;
import swervelib.SwerveInputStream;

// import frc.robot.subsystems.KeyboardInput;

// import java.util.Scanner;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
@SuppressWarnings("unused")
public class RobotContainer {

  // Replace with CommandPS4Controller or CommandJoystick if needed

  public static final CommandXboxController driverXbox = new CommandXboxController(0);
  // final CommandGenericHID genericHID = new CommandGenericHID(1);
  public static final CommandXboxController coralController = new CommandXboxController(1);
  // final CommandGenericHID genericHID = new CommandGenericHID(1);
  // The robot's subsystems and commands are defined here...
  public static final SwerveSubsystem drivebase =
      new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve/neo"));

  private MechanismLigament2d m_elevator;
  private MechanismLigament2d m_wrist;
  private MechanismLigament2d m_wrist2;

  // private final KeyboardInput keyboard;

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

  public static ElevatorCoralSubsystem elevatorCoralSubsystem = new ElevatorCoralSubsystem();
  // public static ClimberSubsystem climberSubsystem = new ClimberSubsystem();
  public static CoralHoldSubsystem coralHoldSubsystem = new CoralHoldSubsystem();
  public static VisionSubsystem visionSubsystem = new VisionSubsystem();
  public static AlgaeSubsystem algaeSubsystem = new AlgaeSubsystem();
  public static CoralWristSubsystem coralWristSubsystem = new CoralWristSubsystem();

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
    // SmartDashboard.putNumber("X-Stop-Dist", -0.26);
    SmartDashboard.putNumber("X-Stop-Dist", -0.09);
    SmartDashboard.putNumber("Y-Stop-Dist", 0.31);
    SmartDashboard.putNumber("yError", 0.017);
    SmartDashboard.putNumber("xError", 0.017);
    SmartDashboard.putNumber("rError", 0.02);

    // keyboard = new KeyboardInput();
    configureBindings();

    // Reset rumble incase its not 0
    driverXbox.setRumble(RumbleType.kBothRumble, 0);

    // Configure the trigger bindings
    DriverStation.silenceJoystickConnectionWarning(true);
    NamedCommands.registerCommand("ElevatorUp", new ElevatorUp(elevatorCoralSubsystem));
    NamedCommands.registerCommand("ElevatorLevel2", new ElevatorMoveLevel2(elevatorCoralSubsystem));
    NamedCommands.registerCommand("ElevatorLevel3", new ElevatorMoveLevel3(elevatorCoralSubsystem));
    NamedCommands.registerCommand("Wait0.25", new WaitCommand(0.25));
    NamedCommands.registerCommand("ElevatorStop", new ElevatorStop(elevatorCoralSubsystem));
    NamedCommands.registerCommand("WristDown", new WristDown(coralWristSubsystem));
    NamedCommands.registerCommand("WristUp", new WristUp(coralWristSubsystem));
    NamedCommands.registerCommand("WristStop", new WristStop(coralWristSubsystem));
    NamedCommands.registerCommand(
        "CoralShootCommand", new CoralShootCommand(coralHoldSubsystem, this));
    NamedCommands.registerCommand("CoralStop", new stopCoralIntake(coralHoldSubsystem));
    NamedCommands.registerCommand("Center", drivebase.centerModulesCommand());
  }

  public void init() {
    Mechanism2d mech = new Mechanism2d(3, 3);
    MechanismRoot2d root = mech.getRoot("climber", 2, 0);
    m_elevator =
        root.append(
            new MechanismLigament2d(
                "elevator", ElevatorSubsystemConstants.elevatorLevels.length, 90));
    m_wrist =
        m_elevator.append(
            new MechanismLigament2d("Coral", 0.5, 90, 6, new Color8Bit(Color.kPurple)));
    m_wrist2 =
        m_elevator.append(
            new MechanismLigament2d("Algae", 0.25, 90, 3, new Color8Bit(Color.kBlue)));
    SmartDashboard.putData("Mech2d", mech);
  }

  public void Periodic() {
    m_elevator.setLength(elevatorCoralSubsystem.getPositionEncoder());
    m_wrist.setAngle(coralWristSubsystem.getEncoder());
    // m_wrist2.setAngle(climberSubsystem.getEncoder());
    // m_wrist2.setAngle(climberSubsystem.getEncoder());

    // String key = keyboard.getLastKeyPressed();

    /*if (key.equalsIgnoreCase("X")) {
      new SequentialCommandGroup(
          new ParallelCommandGroup(
              new ElevatorMoveLevel2(elevatorSubsystem), new ArmLevel2(coralHoldAngleSubsystem)),
          new WaitCommand(1),
          new CoralShootCommand(coralHoldSubsystem));
    } else if (key.equalsIgnoreCase("S")) {
      new SequentialCommandGroup(
          new ParallelCommandGroup(
              new ElevatorMoveLevel3(elevatorSubsystem), new ArmLevel3(coralHoldAngleSubsystem)),
          new WaitCommand(1),
          new CoralShootCommand(coralHoldSubsystem));
    } else if (key.equalsIgnoreCase("W")) {
      new SequentialCommandGroup(
          new ParallelCommandGroup(
              new ElevatorMoveLevel4(elevatorSubsystem), new ArmLevel4(coralHoldAngleSubsystem)),
          new WaitCommand(1),
          new CoralShootCommand(coralHoldSubsystem));
    }*/

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

    // Coral Controller ABYX buttons
    coralController.y().onTrue(new ElevatorMoveLevel3(elevatorCoralSubsystem));
    coralController.b().onTrue(new ElevatorMoveLevel2(elevatorCoralSubsystem));
    coralController.a().onTrue(new ArmSetToMiddle(coralWristSubsystem));
    coralController.x().onTrue(new ArmSetToScore(coralWristSubsystem));

    // Coral Controller D-pad
    coralController.povRight().whileTrue(new GoToCommand(6));
    coralController.povUp().onTrue(
      new SequentialCommandGroup(
          new ElevatorMoveLevel2(elevatorCoralSubsystem),
          new AlignWithNearest(-0.32, coralController.rightBumper(), driverXbox)));
;

    // Coral controller trigger and bumpers
    coralController.rightBumper().onTrue(new CoralShootCommand(coralHoldSubsystem));
    coralController.rightBumper().onFalse(new stopCoralIntake(coralHoldSubsystem));

    // first controller abyx
    driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
    driverXbox.a().whileTrue(drivebase.centerModulesCommand());
    driverXbox.y().onTrue(new AlgaeIntakeCommand(algaeSubsystem));
    driverXbox.b().whileTrue(new AlgaeShootCommand(algaeSubsystem).withTimeout(0.5));
    driverXbox.b().onFalse(new algaeStopIntake(algaeSubsystem));
    driverXbox
        .x()
        .onTrue(
            new SequentialCommandGroup(
                new ElevatorMoveLevel2(elevatorCoralSubsystem),
                new AlignWithNearest(-0.32, coralController.rightBumper(), driverXbox)));

    // first controller d-pad
    driverXbox.povLeft().whileTrue(new CoralIntakeCommand(coralHoldSubsystem));
    driverXbox.povRight().onFalse(new stopCoralIntake(coralHoldSubsystem));
    driverXbox.povUp().onTrue(new WristUp(coralWristSubsystem));
    driverXbox.povDown().onTrue(new WristDown(coralWristSubsystem));
    driverXbox.povUp().onFalse(new WristStop(coralWristSubsystem));
    driverXbox.povDown().onFalse(new WristStop(coralWristSubsystem));
    driverXbox.povRight().onTrue(new CoralShootCommand(coralHoldSubsystem).withTimeout(0.5));
    driverXbox.povRight().onFalse(new stopCoralIntake(coralHoldSubsystem));
    driverXbox.povLeft().onFalse(new stopCoralIntake(coralHoldSubsystem));
    driverXbox.povRight().onFalse(new algaeStopIntake(algaeSubsystem));

    // trigger and bumper
    driverXbox.rightTrigger().onFalse(new ElevatorStop(elevatorCoralSubsystem));
    driverXbox.rightBumper().onTrue(new ArmSetToCoralAccept(coralWristSubsystem));
    driverXbox.leftTrigger().onFalse(new ElevatorStop(elevatorCoralSubsystem));
    driverXbox.leftTrigger().whileTrue(new ElevatorUp(elevatorCoralSubsystem));
    driverXbox.rightTrigger().onTrue(new ElevatorDown(elevatorCoralSubsystem));

    drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);

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

  public static Command getCoralPathCommand(String chooser) {
    Optional<Alliance> ally = DriverStation.getAlliance();
    if (ally.get() == Alliance.Blue) {
      if (chooser.equals("left")) {
        return drivebase.driveToPose(new Pose2d(1.9f, 7.5f, new Rotation2d(Math.toRadians(65))));
      } else {
        return drivebase.driveToPose(new Pose2d(1.9f, 0.5f, new Rotation2d(Math.toRadians(-125))));
      }
    } else {
      if (chooser.equals("left")) {
        return drivebase.driveToPose(new Pose2d(15.9f, 1.5f, new Rotation2d(Math.toRadians(-54))));
      } else {
        return drivebase.driveToPose(new Pose2d(15.8f, 6.5f, new Rotation2d(Math.toRadians(52))));
      }
    }
  }

  /**
   * Sets if YAGSL should put all the motors into brake mode and stop the robot
   *
   * @param brake Should we brake?
   */
  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }
}
