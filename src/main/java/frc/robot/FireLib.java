package frc.robot;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.ArrayList;
import java.util.logging.Level;
import java.util.logging.Logger;

/**
 * Library that holds any portable code to be used in future years. The idea for this came from
 * Mr.Strickland: He saw how we restarted our code base for items like elevators and arms every year
 * He had the idea to create a framework that allows us to easily maintain code to save time!
 */
public class FireLib {

  // Elevator
  public class ElevatorSubsystem {
    private final SparkFlex leftMotor;
    private final SparkFlex rightMotor;
    private final SparkFlexConfig leftMotorConfig;
    private final SparkFlexConfig rightMotorConfig;

    private double MaxPosition;
    private double MinPosition;

    private double ArbUpSpeed;
    private double ArbDownSpeed;

    @SuppressWarnings("unused")
    private RelativeEncoder encoder;

    private float Arbfeedforward;

    private SparkClosedLoopController closedLoopController;

    /** Position measurements from the encoder to which the elevator should goto on command */
    public ArrayList<Double> levels = new ArrayList<>();

    /**
     * Creates an ElevatorSubsystem taken from the code for the 2025 FRC Season ReefScape Please
     * fill in the ArrayList {@link levels} with the levels you wish to hold Call {@link Periodic}
     * in robot Periodic
     *
     * @implNote PLEASE TEST (Not tested YET)
     * @param ELEVATOR_LEFT_MOTOR_ID CANID of the Left Motor
     * @param ELEVATOR_RIGHT_MOTOR_ID CANID of the Right Motor
     * @param idleMode IdleMode that the Elevator Motors should follow (Brake mode recomended)
     * @param PIDF List containing the PIDF values to be sent to the motors
     * @param ArbitraryFeedForward (Part of PIDF...kinda) speed value always sent to the elevator to
     *     hold its position without having to calculate FF
     * @param MaxElevatorPosition The max position of the Elevator that we can get too (If you don't
     *     know set to -1)
     * @param MinElevatorPosition The min position of the Elevator that we can get too (If you don't
     *     know set to 1)
     * @param ArbitraryUpSpeed Speed the Elevator will follow (when going up) if not using PIDF for
     *     position holding
     * @param ArbitraryDownSpeed Speed the Elevator will follow (when going down) if not using PIDF
     *     for position holding
     * @param smartCurrentLimit The smart current limit sent to the motors (default is any where
     *     from 40-50)
     * @param inverted If the Elevator is inverted
     */
    public ElevatorSubsystem(
        int ELEVATOR_LEFT_MOTOR_ID,
        int ELEVATOR_RIGHT_MOTOR_ID,
        IdleMode idleMode,
        double[] PIDF,
        float ArbitraryFeedForward,
        double MaxElevatorPosition,
        double MinElevatorPosition,
        double ArbitraryUpSpeed,
        double ArbitraryDownSpeed,
        int smartCurrentLimit,
        boolean inverted) {
      leftMotor =
          new SparkFlex(
              ELEVATOR_LEFT_MOTOR_ID, MotorType.kBrushless); // Assign motor controller port
      rightMotor =
          new SparkFlex(
              ELEVATOR_RIGHT_MOTOR_ID, MotorType.kBrushless); // Assign motor controller port

      closedLoopController = leftMotor.getClosedLoopController();
      encoder = leftMotor.getEncoder();
      leftMotorConfig = new SparkFlexConfig();
      rightMotorConfig = new SparkFlexConfig();
      leftMotorConfig.encoder.positionConversionFactor(1);
      leftMotorConfig.encoder.velocityConversionFactor(1);
      leftMotorConfig.smartCurrentLimit(smartCurrentLimit);
      rightMotorConfig.smartCurrentLimit(smartCurrentLimit);
      leftMotorConfig.idleMode(idleMode);
      rightMotorConfig.idleMode(idleMode);
      rightMotorConfig.inverted(inverted);
      rightMotorConfig.follow(leftMotor, true);

      leftMotorConfig
          .closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          .pidf(PIDF[0], PIDF[1], PIDF[2], PIDF[3], ClosedLoopSlot.kSlot0);

      Arbfeedforward = ArbitraryFeedForward;
      MaxPosition = MaxElevatorPosition;
      MinPosition = MinElevatorPosition;
      ArbUpSpeed = ArbitraryUpSpeed;
      ArbDownSpeed = ArbitraryDownSpeed;

      leftMotor.configure(
          leftMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
      rightMotor.configure(
          rightMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    /** Updates the Speed and Encoder Position of the Elevator to SmartDashboard */
    public void Periodic() {
      SmartDashboard.putNumber("Elevator-Speed", leftMotor.get());
      SmartDashboard.putNumber("Elevator-EncoderPos", getPositionEncoder());
    }

    /**
     * Get the current speed that the motors are moving at
     *
     * @apiNote The elevator is inverted (-1 goes up and 1 goes down)
     * @return Left motor speed(both motors are mirrored)
     */
    public double getSpeed() {
      return leftMotor.get();
    }

    /**
     * Set the level that the elevator should move too Uses the ArbitraryUpSpeed and
     * ArbitraryDownSpeed to set the speed to follow
     *
     * @apiNote Currently uses {@link #goToLevelNoPID(double)} which will be shakey due to not using
     *     PIDF
     */
    public void setLevel(int level) {
      if (level < 0 || level >= levels.size()) {
        System.out.println("Invalid level: " + level);
        return;
      }
      goToLevelNoPID(levels.get(level), ArbUpSpeed, ArbDownSpeed);
    }

    /**
     * Sets the speed of the elevator with some safe guards (prevented from moving outside of max
     * and min values). And holds the position using the flat ArbitraryFeedForward
     *
     * @param speed the speed that the motors should be commanded too
     */
    public void setSpeed(double speed) {
      if (MaxPosition != -1 && MinPosition != 1) {
        if (getPositionEncoder() >= MaxPosition && speed > 0) {
          leftMotor.set(0);
          closedLoopController.setReference(
              0, ControlType.kPosition, ClosedLoopSlot.kSlot0, Arbfeedforward);
          Logger.getGlobal().log(Level.WARNING, "Elevator Trying to move past maximums");
        } else if (getPositionEncoder() <= MinPosition && speed < 0) {
          leftMotor.set(0);
          closedLoopController.setReference(
              0, ControlType.kPosition, ClosedLoopSlot.kSlot0, Arbfeedforward);
          Logger.getGlobal().log(Level.WARNING, "Elevator Trying to move past minimums");
        } else {
          leftMotor.set(speed);
        }
      }
    }

    /**
     * Logs the encoder value currently to logs and then the value we are looking for in logs
     *
     * @param index of level we are looking for
     */
    public boolean atLevel(int levlNeeded) {
      Logger.getGlobal().log(Level.INFO, "looking for level");
      Logger.getGlobal().log(Level.INFO, "encoder value " + getPositionEncoder());
      Logger.getGlobal().log(Level.INFO, "target value " + levels.get(levlNeeded));

      return getPositionEncoder() < levels.get(levlNeeded);
    }

    /**
     * Goes to a setpoint using if statements. (No PIDF) But will hold using PIDF
     * (ArbitraryFeedForward)
     *
     * @param setPoint relative encoder position desired
     */
    public void goToLevelNoPID(double setPoint, double arbUp, double arbDown) {
      // double setPoint = -39;
      if (setPoint + 1 >= getPositionEncoder()) {
        leftMotor.set(arbUp);
        Logger.getGlobal().log(Level.INFO, "Going Down");
      }
      if (setPoint - 1 <= getPositionEncoder()) {
        leftMotor.set(arbDown);
        Logger.getGlobal().log(Level.INFO, "Going Up");
      }
      if (setPoint - 2 >= getPositionEncoder() && setPoint + 2 <= getPositionEncoder()) {
        Logger.getGlobal().log(Level.INFO, "Found Level");
        leftMotor.set(0);
        closedLoopController.setReference(
            getPositionEncoder(), ControlType.kPosition, ClosedLoopSlot.kSlot0, Arbfeedforward);
      }
    }

    /**
     * Move to a position for the elevator to move to using PIDF.
     *
     * @param position the encoder position to move too
     */
    public void moveToPosition(double position) {
      closedLoopController.setReference(
          position, ControlType.kPosition, ClosedLoopSlot.kSlot0, Arbfeedforward);
    }

    /** Stop the elevator from moving and hold the position with the flat feed forward */
    public void stop() {
      leftMotor.set(0);

      closedLoopController.setReference(
          getPositionEncoder(), ControlType.kPosition, ClosedLoopSlot.kSlot0, Arbfeedforward);
    }

    /**
     * Get the position of the elevator encoder
     *
     * @return Position of the left motor encoder(the right motor follows the left so it doesn't
     *     matter)
     */
    public double getPositionEncoder() {
      return leftMotor.getEncoder().getPosition();
    }

    /**
     * If the elevator is finished moving to a position If we are in the sim blanket return 0 as we
     * cannot move a mechanism
     */
    public boolean isFinished(int position) {
      if (Robot.isSimulation()) {
        return true;
      }
      if (levels.get(position) - (leftMotor.getEncoder().getPosition()) == 0) {
        return true;
      } else {
        return false;
      }
    }
  }

  // Arm
  public class ArmSubsystem {
    private final SparkFlex motor;
    private final SparkFlexConfig motorConfig;

    private double wantedPos;

    SparkClosedLoopController controller;

    private boolean buttonPressed;

    private TrapezoidProfile.State state;

    private ArmFeedforward feedforward;

    public static final double shooter = 4.82;
    public double finalPos;

    private double MoveSpeed;
    private String SmartDashboardNickName;

    /** Angle measurements from the encoder to which the arm should goto on command */
    public ArrayList<Double> levels = new ArrayList<>();

    /**
     * Creates an ArmSubsystem place {@link #periodic()} into robot periodic
     *
     * @see ArmFeedforward For helping setting up the feedforward
     * @param ARM_MOTOR_ID CANID of the Arm motor
     * @param idleMode IdleMode that the Arm motor should follow (Brake mode recomended)
     * @param PIDF Double list containing PIDF values
     * @param feedforward The feedforward settings for the arm
     * @param MoveSpeed Move speed for the arm (used by {@link #tiltDown()} and {@link #tiltUp()})
     * @param SmartDashboardNickName The name of this arm given when putting values to
     *     SmartDashboard
     */
    public ArmSubsystem(
        int ARM_MOTOR_ID,
        IdleMode idleMode,
        Double[] PIDF,
        ArmFeedforward feedforward,
        double MoveSpeed,
        String SmartDashboardNickName) {
      motor = new SparkFlex(ARM_MOTOR_ID, MotorType.kBrushless); // Assign motor controller port

      controller = motor.getClosedLoopController();
      motorConfig = new SparkFlexConfig();

      this.feedforward = feedforward;

      motorConfig.idleMode(idleMode);
      motorConfig
          .closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          .pidf(PIDF[0], PIDF[1], PIDF[2], PIDF[3], ClosedLoopSlot.kSlot0);

      motor.configure(
          motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
      // encoder = new Encoder(1, 1); // Assign encoder ports
      wantedPos = motor.getEncoder().getPosition();
      state = new State(wantedPos, 0);

      this.MoveSpeed = MoveSpeed;
      this.SmartDashboardNickName = SmartDashboardNickName;
    }

    /** Updates the Speed and Encoder position/angle of the Arm to SmartDashboard */
    public void periodic() {
      SmartDashboard.putNumber(SmartDashboardNickName + "Encoder:", getPositionEncoder());

      if (!buttonPressed) {
        wantedPos = getPositionEncoder();
        state = new State(wantedPos, 0);
        holdUp(state);
      }
    }

    /** Free hand tilt down. Just hold a button and go. Moves at MoveSpeed */
    public void tiltDown() {
      buttonPressed = true;

      Logger.getGlobal().log(Level.INFO, "DOWN " + getPositionEncoder());
      motorConfig.inverted(true);
      motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

      motor.set(MoveSpeed);
    }

    /**
     * Move the arm to Intake position at MoveSpeed When at the position hold it with {@link
     * #holdUp(State)}
     *
     * @apiNote DOESN'T USE PIDF
     * @param setPoint The angle that we should be at to intake
     * @implNote Both years I have been on the team we have had a way for Human Player to enter Game
     *     Objects onto the field. This function is designed for you to be able to reuse that
     *     functionality easily
     */
    public void IntakePosition(double setPoint) {
      if (setPoint + 0.1 < getPositionEncoder()) {
        motorConfig.inverted(false);
        motor.set(MoveSpeed);
      } else if (setPoint - 0.1 > getPositionEncoder()) {
        motorConfig.inverted(true);
        motor.set(MoveSpeed);
      } else {
        wantedPos = getPositionEncoder();
        state = new State(wantedPos, 0);
        holdUp(state);
      }
    }

    /** Free hand tilt up. Just hold a button and go. Moves at MoveSpeed */
    public void tiltUp() {
      buttonPressed = true;

      Logger.getGlobal().log(Level.INFO, "UP: " + getPositionEncoder());

      motorConfig.inverted(false);
      motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

      motor.set(MoveSpeed);
    }

    /**
     * Gets the motor encoder position of the arm
     *
     * @return Encoder Position
     */
    public double getEncoder() {
      return motor.getEncoder().getPosition();
    }

    /**
     * Gets the dedicated relative encoder position
     *
     * @return
     */
    private double getPositionEncoder() {
      return motor.getEncoder().getPosition();
    }

    /**
     * Goes to a level index using PIDF
     *
     * @param levelNeeded Index of level to goto
     */
    public void setLevel(int levelNeeded) {

      double setPoint = levels.get(levelNeeded);

      double ff = feedforward.calculate(setPoint * 2 * Math.PI, 0);
      controller.setReference(setPoint, ControlType.kPosition, ClosedLoopSlot.kSlot0, ff);
    }

    /**
     * Logs the encoder value currently to logs and then the value we are looking for in logs
     *
     * @param index of level we are looking for
     */
    public boolean atLevel(int levlNeeded) {
      Logger.getGlobal().log(Level.INFO, "looking for level");
      Logger.getGlobal().log(Level.INFO, "encoder value " + getPositionEncoder());
      Logger.getGlobal().log(Level.INFO, "target value " + levels.get(levlNeeded));

      return getPositionEncoder() < levels.get(levlNeeded);
    }

    /**
     * Returns if we have reached position
     *
     * @param position to check if we have reached
     */
    public boolean isFinished(int position) {

      if (shooter - (motor.getEncoder().getPosition()) == 0) {
        return true;
      } else {
        return false;
      }
    }

    /**
     * Uses the ff calculator to keep the arm in a single place
     *
     * @param TrapezoidProfile.State Setpoint position to hold
     */
    public void holdUp(TrapezoidProfile.State setpoint) {
      double ff = feedforward.calculate(setpoint.position * 2 * Math.PI, setpoint.velocity);
      controller.setReference(0, ControlType.kPosition, ClosedLoopSlot.kSlot0, ff);
    }

    /** Stops the Tilting and also holds using PIDF/{@link #holdUp(TrapezoidProfile.State)} */
    public void stopTilt() {
      motor.set(0);

      State setPoint = new State(getPositionEncoder(), 0);
      holdUp(setPoint);
      buttonPressed = false;
    }
  }

  // Game object end effector
  public class endEffectorSubsystem {
    private final SparkFlex motor;
    private final SparkFlexConfig motorConfig;
    private double IntakeSpeed, ShootSpeed;

    private DigitalInput limitSwitch;

    /**
     * Creates a subsystem to control an end effector Designed for end effectors that have
     * mechanisms that directly grab the object and can shoot it
     *
     * @param MOTOR_ID CANID of motor to control
     * @param CurrentLimit Smart current limit of motor. A good value would be (40-50)
     * @param inverted If the motor is inverted
     * @param PIDF A double list containing PIDF values
     * @param IntakeSpeed The speed to intake at
     * @param ShootSpeed The speed to shoot at
     * @param DIOLimitSwitch Digital input limit switch
     */
    public endEffectorSubsystem(
        int MOTOR_ID,
        int CurrentLimit,
        boolean inverted,
        double[] PIDF,
        double IntakeSpeed,
        double ShootSpeed,
        DigitalInput DIOLimitSwitch) {
      motor = new SparkFlex(MOTOR_ID, MotorType.kBrushless);
      motorConfig = new SparkFlexConfig();

      motorConfig.smartCurrentLimit(CurrentLimit);
      motorConfig.inverted(inverted);

      motorConfig
          .closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          .pidf(PIDF[0], PIDF[1], PIDF[2], PIDF[3], ClosedLoopSlot.kSlot0);

      motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      this.IntakeSpeed = IntakeSpeed;
      this.ShootSpeed = ShootSpeed;
      limitSwitch = DIOLimitSwitch;
    }

    /**
     * Creates a subsystem to control an end effector Designed for end effectors that have
     * mechanisms that directly grab the object and can shoot it
     *
     * @param MOTOR_ID CANID of motor to control
     * @param CurrentLimit Smart current limit of motor. A good value would be (40-50)
     * @param inverted If the motor is inverted
     * @param PIDF A double list containing PIDF values
     * @param IntakeSpeed The speed to intake at
     * @param ShootSpeed The speed to shoot at
     */
    public endEffectorSubsystem(
        int MOTOR_ID,
        int CurrentLimit,
        boolean inverted,
        double[] PIDF,
        double IntakeSpeed,
        double ShootSpeed) {
      motor = new SparkFlex(MOTOR_ID, MotorType.kBrushless);
      motorConfig = new SparkFlexConfig();

      motorConfig.smartCurrentLimit(CurrentLimit);
      motorConfig.inverted(inverted);

      motorConfig
          .closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          .pidf(PIDF[0], PIDF[1], PIDF[2], PIDF[3], ClosedLoopSlot.kSlot0);

      motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      this.IntakeSpeed = IntakeSpeed;
      this.ShootSpeed = ShootSpeed;
    }

    /** Shoots the motor at {@link #ShootSpeed} */
    public void DirectShoot() {
      motor.set(ShootSpeed); // Run motor forward to shoot
    }

    /**
     * Plagiarized from ChatGPT as I have not taken a physics class yet. Solves for a valid velocity
     * vector to hit a target, using a fixed angle. Returns null if the input angle is not correct
     * or there is no correct solution. (Will also send WARNINGs to the DS)
     *
     * @param target Target point in space (Robot space for easier math)
     * @param launchAngleDeg The fixed angle we are shooting from
     * @param maxLaunchSpeed Max velocity that the game object can be launched at
     * @return [Motor power, Angle]
     */
    public static double[] calculateLaunchVelocity(
        Translation3d target, double launchAngleDeg, double maxLaunchSpeed) {
      double dx =
          Math.sqrt(
              target.getX() * target.getX() + target.getZ() * target.getZ()); // Horizontal distance
      double dy = target.getY();

      if (dx == 0) return null; // Prevent divide-by-zero
      // Use fixed launch angle
      double angleRad = Math.toRadians(launchAngleDeg);
      double cos = Math.cos(angleRad);

      // Time to target (quadratic from vertical motion)
      double vSquared = (9.81 * dx * dx) / (2 * cos * cos * (dx * Math.tan(angleRad) - dy));

      if (vSquared <= 0) {
        Logger.getGlobal()
            .log(
                Level.WARNING,
                "Cannot shoot to (Unable to break laws of physics): " + target.toString());
        return null; // Not physically possible
      }

      double v = Math.sqrt(vSquared);
      if (v > maxLaunchSpeed) {
        Logger.getGlobal()
            .log(Level.WARNING, "Cannot shoot to (Not enough power): " + target.toString());
        return null; // Need more power than available
      }

      // Motor power as a fraction of max speed
      double power = v / maxLaunchSpeed;

      return new double[] {power, launchAngleDeg}; // Return motor power and fixed launch angle
    }

    /**
     * Plagiarized from ChatGPT as I have not taken a physics class yet Solves for a valid velocity
     * vector to hit a target. Returns null if no valid solution exists. (Will also send WARNINGs to
     * the DS)
     *
     * @param target Target point in space (Robot space for easier math)
     * @param maxLaunchSpeed Max velocity that the game object can be launched at
     * @param maxAngle Highest angle that the end effector can point at
     * @param minAngle Lowest angle that the end effector can point at
     * @return [Motor power, Angle]
     */
    public static double[] calculateLaunchVelocity(
        Translation3d target, double maxLaunchSpeed, double maxAngle, double minAngle) {
      double dx =
          Math.sqrt(
              target.getX() * target.getX() + target.getZ() * target.getZ()); // Horizontal distance
      double dy = target.getY();

      if (dx == 0) return null; // Prevent divide-by-zero
      // Solve for angle and velocity
      double g = 9.81;
      double v2_min = Double.POSITIVE_INFINITY;
      double bestPower = -1;
      double bestAngle = -1;

      for (double angle = minAngle; angle <= maxAngle; angle += 0.5) {
        double rad = Math.toRadians(angle);
        double cos = Math.cos(rad);

        double vSquared = (g * dx * dx) / (2 * cos * cos * (dx * Math.tan(rad) - dy));

        if (vSquared > 0) {
          double v = Math.sqrt(vSquared);
          if (v <= maxLaunchSpeed && vSquared < v2_min) {
            // Save best (lowest required v²)
            v2_min = vSquared;

            // Store power and angle
            bestPower = v / maxLaunchSpeed; // Motor power
            bestAngle = angle;
          }
        }
      }

      if (bestPower == -1) {
        Logger.getGlobal()
            .log(
                Level.WARNING,
                "Cannot find valid solution to shoot (tried the following angles): "
                    + minAngle
                    + " to "
                    + maxAngle);
        return null; // No valid solution
      }

      return new double[] {bestPower, bestAngle}; // Return motor power and launch angle
    }

    /**
     * Intakes the motor at {@link #IntakeSpeed}. Stops when the limit switch detects the object (if
     * used)
     */
    public void intake() {
      if (limitSwitch != null) {
        if (!limitSwitch.get()) {
          motor.set(IntakeSpeed);
        } else {
          motor.set(0);
        }
      } else {
        motor.set(IntakeSpeed);
      }
    }

    /** Stops the motor */
    public void stop() {
      motor.set(0);
    }
  }
}
