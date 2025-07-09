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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.logging.Level;
import java.util.logging.Logger;

/**
 * Library that holds any portable code to be used in future years. The idea for this came from
 * Mr.Strickland: He saw how we restarted our code base for items like elevators and arms every year
 * He had the idea to create a framework that allows us to easily maintain code to save time!
 *
 * <p>Intended to act as a base to replace SubsystemBase in the matching subsystem.
 *
 * <p>This library is documented where the names or control flow isn't evident (Please document your
 * changes as such) and is intended to be changed to be improved upon.
 */
public final class FireLib {

  /**
   * As far as I know there isn't a built in GCD/GCF operation and I need it So I copy and pasted
   * from google :p
   */
  public static int gcd(int a, int b) {
    if (b == 0) return a;
    return gcd(b, a % b);
  }

  // 2 motor Elevator
  public static class DuelMotorElevatorSubsystemBase extends SubsystemBase {

    private String name; // Nickname to be used in Smartdashboard messages

    private MechanismLigament2d m_elevator; // The visulizer version of this elevator

    public final SparkFlex leftMotor;
    public final SparkFlex rightMotor;
    public final SparkFlexConfig leftMotorConfig;
    public final SparkFlexConfig rightMotorConfig;

    private double MaxPosition;
    private double MinPosition;

    private boolean inverted;

    // The arbitrary speed to be used up and down without PIDF
    private double ArbUpSpeed;
    private double ArbDownSpeed;

    @SuppressWarnings("unused")
    private RelativeEncoder encoder;

    // The arbitrary value that is always sent to hold the elevator up without PIDF
    private float Arbfeedforward;

    // The closed loop controller on the sparkflex/max
    private SparkClosedLoopController closedLoopController;

    /** Position measurements from the encoder to which the elevator should goto on command */
    public ArrayList<Double> levels = new ArrayList<>();

    /**
     * Creates an ElevatorSubsystem taken from the code for the 2025 FRC Season ReefScape Please
     * fill in the ArrayList {@link levels} with the levels you wish to hold Place periodic in robot
     * periodic
     *
     * @implNote PLEASE TEST (Not tested YET)
     * @param name Nickname of this elevator
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
    public DuelMotorElevatorSubsystemBase(
        String name,
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

      // Getting a reference to the closed loop controller and encoder on the sparkflex/max
      closedLoopController = leftMotor.getClosedLoopController();
      encoder = leftMotor.getEncoder();

      // Getting a reference to the config of the sparkflex/max
      leftMotorConfig = new SparkFlexConfig();
      rightMotorConfig = new SparkFlexConfig();

      // Setting the conversion factors (1:1 in this case)
      leftMotorConfig.encoder.positionConversionFactor(1);
      leftMotorConfig.encoder.velocityConversionFactor(1);

      leftMotorConfig.smartCurrentLimit(smartCurrentLimit);
      rightMotorConfig.smartCurrentLimit(smartCurrentLimit);
      leftMotorConfig.idleMode(idleMode);
      rightMotorConfig.idleMode(idleMode);
      rightMotorConfig.inverted(inverted);

      // We run the elevator as right motor dominant
      rightMotorConfig.follow(leftMotor, true);

      this.name = name;

      this.inverted = inverted;

      // If PIDF is a thing set it in the motor configs using the encoder as feedback
      // or just tell it to use the encoder and disable PIDF
      if (PIDF != null) {
        leftMotorConfig
            .closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pidf(PIDF[0], PIDF[1], PIDF[2], PIDF[3], ClosedLoopSlot.kSlot0);
      } else {
        leftMotorConfig
            .closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pidf(0, 0, 0, 0, ClosedLoopSlot.kSlot0);
      }

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

    /**
     * Does the setup for the visulizer interpretation of this elevator Doesn't put the data to
     * SmartDashboard (Do that when your done adding ligaments)
     *
     * @param root The root object to attach this elevator too
     * @param angle The angle this is elevator is at relative to the root obj (0 is pointing towards
     *     the front, 90 is pointing up)
     */
    public MechanismLigament2d setupVisulizer(MechanismRoot2d root, double angle) {

      // adds the elevator onto the root obj with a max length of 1 and sets the angle relative to
      // root
      m_elevator = root.append(new MechanismLigament2d("elevator", 1, angle));

      // Then return the created obj
      return m_elevator;
    }

    /**
     * Does the setup for the visulizer interpretation of this elevator Doesn't put the data to
     * SmartDashboard (Do that when your done adding ligaments)
     *
     * @param root The obj to attach this elevator too
     * @param angle The angle this is elevator is at relative to its attached obj
     */
    public MechanismLigament2d setupVisulizer(MechanismLigament2d root, double angle) {
      // adds the elevator onto the parent obj with a max length of 1 and sets the angle relative to
      // the parent
      m_elevator = root.append(new MechanismLigament2d("elevator", 1, angle));

      // Then return the created obj
      return m_elevator;
    }

    /**
     * Attaches a Mechanism to this elevators visulization
     *
     * @param Mechanism to be attached
     * @return Elevator with attached ligament
     */
    public MechanismLigament2d AttachMechanism(MechanismLigament2d Mechanism) {
      return m_elevator.append(Mechanism);
    }

    /** Updates the Speed and Encoder Position of the Elevator to SmartDashboard */
    public void Periodic() {
      SmartDashboard.putNumber(name + "-Speed", leftMotor.get());
      SmartDashboard.putNumber(name + "-EncoderPos", getPositionEncoder());

      // If we have an elevator
      if (m_elevator != null) {

        // and we are not in the sim
        if (!Robot.isSimulation()) {

          // If we are not inverted and have a max position
          if (!inverted && MaxPosition != -1) {

            // divide the encoder position by the max to get a value between 0-1
            m_elevator.setLength(getPositionEncoder() / MaxPosition);

            // Else we are inverted and should use the minimum position to do the same
            // (absolute the values to prevent weird math)
          } else if (MinPosition != 1) {
            m_elevator.setLength(Math.abs(getPositionEncoder()) / Math.abs(MinPosition));
          }
          // if all else fails we don't have a max/min value (Making it info rather then warning as
          // this runs every tick)
          else {
            Logger.getGlobal().log(Level.INFO, name + "-Visulizer: Missing max/min position");
          }

          // if we are in the simulator
        } else {
          // I don't know how long it takes for your elevator to reach its peak
          // so I added a stand in while in the sim. Some feedback is better then no feedback
          m_elevator.setLength(m_elevator.getLength() + (getSpeed() / 10));
        }
      }
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
     * Set the level that the elevator should move too. Uses the ArbitraryUpSpeed and
     * ArbitraryDownSpeed to set the speed to follow
     *
     * @apiNote Currently uses {@link #goToLevelNoPID(double)} which will be shakey due to not using
     *     PIDF
     */
    public void setLevel(int level) {
      // check to make sure the level is above 0 and not calling for level 99999999
      if (level < 0 || level >= levels.size()) {
        // if its invalid call it out
        Logger.getGlobal().log(Level.WARNING, name + ": Invalid level " + level);
        return;
      }

      // else were okay to ask the elevator to move to that level
      goToLevelNoPID(levels.get(level), ArbUpSpeed, ArbDownSpeed);
    }

    /**
     * Sets the speed of the elevator with some safe guards*. And holds the position using the flat
     * ArbitraryFeedForward
     *
     * <p>* prevented from moving outside of max and min values and only if max and min values exist
     *
     * @param speed the speed that the motors should be commanded too
     */
    public void setSpeed(double speed) {

      // Check to see if we have max and min position values
      if (MaxPosition != -1 && MinPosition != 1) {

        // If we are beyond the max position do not move up
        if (getPositionEncoder() >= MaxPosition && speed > 0) {
          leftMotor.set(0);

          // tell PIDF to hold it where we are
          closedLoopController.setReference(
              0, ControlType.kPosition, ClosedLoopSlot.kSlot0, Arbfeedforward);

          // phone home
          Logger.getGlobal().log(Level.WARNING, name + ": Trying to move past maximums");

          // If we are beyond the min position do not move down
        } else if (getPositionEncoder() <= MinPosition && speed < 0) {
          leftMotor.set(0);

          // tell PIDF to hold it where we are
          closedLoopController.setReference(
              0, ControlType.kPosition, ClosedLoopSlot.kSlot0, Arbfeedforward);

          // phone home
          Logger.getGlobal().log(Level.WARNING, name + ": Trying to move past minimums");

          // If other checks didn't pass we are free to move as requested!
        } else {
          leftMotor.set(speed);
        }

        // if we don't have max and min positions just set the speed to the requested
      } else {
        leftMotor.set(speed);
      }
    }

    /**
     * Logs the encoder value currently to logs and then the value we are looking for in logs
     *
     * @param index of level we are looking for
     */
    public boolean atLevel(int levlNeeded) {
      Logger.getGlobal().log(Level.INFO, name + ": looking for level");
      Logger.getGlobal().log(Level.INFO, name + ": encoder value " + getPositionEncoder());
      Logger.getGlobal().log(Level.INFO, name + ": target value " + levels.get(levlNeeded));

      return getPositionEncoder() < levels.get(levlNeeded);
    }

    /**
     * Goes to a setpoint using if statements. (No PIDF) But will hold using PIDF
     * (ArbitraryFeedForward)
     *
     * @param setPoint relative encoder position desired
     */
    public void goToLevelNoPID(double setPoint, double arbUp, double arbDown) {

      // changed this a bit as I found it was doing some weird stuff. Everything was its own If
      // statement
      // and: "setPoint + 1 >= getPositionEncoder()"
      // If there is something I'm missing........

      // If within 2 units of setpoint hold position using arbFF
      if (setPoint - 2 >= getPositionEncoder() && setPoint + 2 <= getPositionEncoder()) {
        Logger.getGlobal().log(Level.INFO, name + ": Found Level");
        leftMotor.set(0);
        closedLoopController.setReference(
            getPositionEncoder(), ControlType.kPosition, ClosedLoopSlot.kSlot0, Arbfeedforward);
      }

      // if greater then move up
      else if (setPoint >= getPositionEncoder()) {
        leftMotor.set(arbUp);
        Logger.getGlobal().log(Level.INFO, name + ": Going Up");
      }

      // if less then move down
      else if (setPoint <= getPositionEncoder()) {
        leftMotor.set(arbDown);
        Logger.getGlobal().log(Level.INFO, name + ": Going Down");
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

      // if we are in the sim. we're always done
      if (Robot.isSimulation()) {
        return true;
      }

      // If we are at our desired level then we are done
      if (levels.get(position) - (leftMotor.getEncoder().getPosition()) == 0) {
        return true;
      } else {
        return false;
      }
    }
  }

  // 2 motor Elevator
  public static class singleMotorElevatorSubsystemBase extends SubsystemBase {

    private String name; // Nickname to be used in Smartdashboard messages

    private MechanismLigament2d m_elevator; // The visulizer version of this elevator

    public final SparkFlex motor;
    public final SparkFlexConfig motorConfig;

    private double MaxPosition;
    private double MinPosition;

    private boolean inverted;

    // The arbitrary speed to be used up and down without PIDF
    private double ArbUpSpeed;
    private double ArbDownSpeed;

    @SuppressWarnings("unused")
    private RelativeEncoder encoder;

    // The arbitrary value that is always sent to hold the elevator up without PIDF
    private float Arbfeedforward;

    // The closed loop controller on the sparkflex/max
    private SparkClosedLoopController closedLoopController;

    /** Position measurements from the encoder to which the elevator should goto on command */
    public ArrayList<Double> levels = new ArrayList<>();

    /**
     * Creates an ElevatorSubsystem taken from the code for the 2025 FRC Season ReefScape Please
     * fill in the ArrayList {@link levels} with the levels you wish to hold Place periodic in robot
     * periodic
     *
     * @implNote PLEASE TEST (Not tested YET)
     * @param name Nickname of this elevator
     * @param ELEVATOR_MOTOR_ID CANID of the Motor
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
    public singleMotorElevatorSubsystemBase(
        String name,
        int ELEVATOR_MOTOR_ID,
        IdleMode idleMode,
        double[] PIDF,
        float ArbitraryFeedForward,
        double MaxElevatorPosition,
        double MinElevatorPosition,
        double ArbitraryUpSpeed,
        double ArbitraryDownSpeed,
        int smartCurrentLimit,
        boolean inverted) {

      motor =
          new SparkFlex(ELEVATOR_MOTOR_ID, MotorType.kBrushless); // Assign motor controller port

      // Getting a reference to the closed loop controller and encoder on the sparkflex/max
      closedLoopController = motor.getClosedLoopController();
      encoder = motor.getEncoder();

      // Getting a reference to the config of the sparkflex/max
      motorConfig = new SparkFlexConfig();

      // Setting the conversion factors (1:1 in this case)
      motorConfig.encoder.positionConversionFactor(1);
      motorConfig.encoder.velocityConversionFactor(1);

      motorConfig.smartCurrentLimit(smartCurrentLimit);
      motorConfig.idleMode(idleMode);
      motorConfig.inverted(inverted);

      this.name = name;

      this.inverted = inverted;

      // If PIDF is a thing set it in the motor configs using the encoder as feedback
      // or just tell it to use the encoder and disable PIDF
      if (PIDF != null) {
        motorConfig
            .closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pidf(PIDF[0], PIDF[1], PIDF[2], PIDF[3], ClosedLoopSlot.kSlot0);
      } else {
        motorConfig
            .closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pidf(0, 0, 0, 0, ClosedLoopSlot.kSlot0);
      }

      Arbfeedforward = ArbitraryFeedForward;
      MaxPosition = MaxElevatorPosition;
      MinPosition = MinElevatorPosition;
      ArbUpSpeed = ArbitraryUpSpeed;
      ArbDownSpeed = ArbitraryDownSpeed;

      motor.configure(
          motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    /**
     * Does the setup for the visulizer interpretation of this elevator Doesn't put the data to
     * SmartDashboard (Do that when your done adding ligaments)
     *
     * @param root The root object to attach this elevator too
     * @param angle The angle this is elevator is at relative to the root obj (0 is pointing towards
     *     the front, 90 is pointing up)
     */
    public MechanismLigament2d setupVisulizer(MechanismRoot2d root, double angle) {

      // adds the elevator onto the root obj with a max length of 1 and sets the angle relative to
      // root
      m_elevator = root.append(new MechanismLigament2d("elevator", 1, angle));

      // Then return the created obj
      return m_elevator;
    }

    /**
     * Does the setup for the visulizer interpretation of this elevator Doesn't put the data to
     * SmartDashboard (Do that when your done adding ligaments)
     *
     * @param root The obj to attach this elevator too
     * @param angle The angle this is elevator is at relative to its attached obj
     */
    public MechanismLigament2d setupVisulizer(MechanismLigament2d root, double angle) {
      // adds the elevator onto the parent obj with a max length of 1 and sets the angle relative to
      // the parent
      m_elevator = root.append(new MechanismLigament2d("elevator", 1, angle));

      // Then return the created obj
      return m_elevator;
    }

    /**
     * Attaches a Mechanism to this elevators visulization
     *
     * @param Mechanism to be attached
     * @return Elevator with attached ligament
     */
    public MechanismLigament2d AttachMechanism(MechanismLigament2d Mechanism) {
      return m_elevator.append(Mechanism);
    }

    /** Updates the Speed and Encoder Position of the Elevator to SmartDashboard */
    public void Periodic() {
      SmartDashboard.putNumber(name + "-Speed", motor.get());
      SmartDashboard.putNumber(name + "-EncoderPos", getPositionEncoder());

      // If we have an elevator
      if (m_elevator != null) {

        // and we are not in the sim
        if (!Robot.isSimulation()) {

          // If we are not inverted and have a max position
          if (!inverted && MaxPosition != -1) {

            // divide the encoder position by the max to get a value between 0-1
            m_elevator.setLength(getPositionEncoder() / MaxPosition);

            // Else we are inverted and should use the minimum position to do the same
            // (absolute the values to prevent weird math)
          } else if (MinPosition != 1) {
            m_elevator.setLength(Math.abs(getPositionEncoder()) / Math.abs(MinPosition));
          }
          // if all else fails we don't have a max/min value (Making it info rather then warning as
          // this runs every tick)
          else {
            Logger.getGlobal().log(Level.INFO, name + "-Visulizer: Missing max/min position");
          }

          // if we are in the simulator
        } else {
          // I don't know how long it takes for your elevator to reach its peak
          // so I added a stand in while in the sim. Some feedback is better then no feedback
          m_elevator.setLength(m_elevator.getLength() + (getSpeed() / 10));
        }
      }
    }

    /**
     * Get the current speed that the motors are moving at
     *
     * @apiNote The elevator is inverted (-1 goes up and 1 goes down)
     * @return Left motor speed(both motors are mirrored)
     */
    public double getSpeed() {
      return motor.get();
    }

    /**
     * Set the level that the elevator should move too. Uses the ArbitraryUpSpeed and
     * ArbitraryDownSpeed to set the speed to follow
     *
     * @apiNote Currently uses {@link #goToLevelNoPID(double)} which will be shakey due to not using
     *     PIDF
     */
    public void setLevel(int level) {
      // check to make sure the level is above 0 and not calling for level 99999999
      if (level < 0 || level >= levels.size()) {
        // if its invalid call it out
        Logger.getGlobal().log(Level.WARNING, name + ": Invalid level " + level);
        return;
      }

      // else were okay to ask the elevator to move to that level
      goToLevelNoPID(levels.get(level), ArbUpSpeed, ArbDownSpeed);
    }

    /**
     * Sets the speed of the elevator with some safe guards*. And holds the position using the flat
     * ArbitraryFeedForward
     *
     * <p>* prevented from moving outside of max and min values and only if max and min values exist
     *
     * @param speed the speed that the motors should be commanded too
     */
    public void setSpeed(double speed) {

      // Check to see if we have max and min position values
      if (MaxPosition != -1 && MinPosition != 1) {

        // If we are beyond the max position do not move up
        if (getPositionEncoder() >= MaxPosition && speed > 0) {
          motor.set(0);

          // tell PIDF to hold it where we are
          closedLoopController.setReference(
              0, ControlType.kPosition, ClosedLoopSlot.kSlot0, Arbfeedforward);

          // phone home
          Logger.getGlobal().log(Level.WARNING, name + ": Trying to move past maximums");

          // If we are beyond the min position do not move down
        } else if (getPositionEncoder() <= MinPosition && speed < 0) {
          motor.set(0);

          // tell PIDF to hold it where we are
          closedLoopController.setReference(
              0, ControlType.kPosition, ClosedLoopSlot.kSlot0, Arbfeedforward);

          // phone home
          Logger.getGlobal().log(Level.WARNING, name + ": Trying to move past minimums");

          // If other checks didn't pass we are free to move as requested!
        } else {
          motor.set(speed);
        }

        // if we don't have max and min positions just set the speed to the requested
      } else {
        motor.set(speed);
      }
    }

    /**
     * Logs the encoder value currently to logs and then the value we are looking for in logs
     *
     * @param index of level we are looking for
     */
    public boolean atLevel(int levlNeeded) {
      Logger.getGlobal().log(Level.INFO, name + ": looking for level");
      Logger.getGlobal().log(Level.INFO, name + ": encoder value " + getPositionEncoder());
      Logger.getGlobal().log(Level.INFO, name + ": target value " + levels.get(levlNeeded));

      return getPositionEncoder() < levels.get(levlNeeded);
    }

    /**
     * Goes to a setpoint using if statements. (No PIDF) But will hold using PIDF
     * (ArbitraryFeedForward)
     *
     * @param setPoint relative encoder position desired
     */
    public void goToLevelNoPID(double setPoint, double arbUp, double arbDown) {

      // changed this a bit as I found it was doing some weird stuff. Everything was its own If
      // statement
      // and: "setPoint + 1 >= getPositionEncoder()"
      // If there is something I'm missing........

      // If within 2 units of setpoint hold position using arbFF
      if (setPoint - 2 >= getPositionEncoder() && setPoint + 2 <= getPositionEncoder()) {
        Logger.getGlobal().log(Level.INFO, name + ": Found Level");
        motor.set(0);
        closedLoopController.setReference(
            getPositionEncoder(), ControlType.kPosition, ClosedLoopSlot.kSlot0, Arbfeedforward);
      }

      // if greater then move up
      else if (setPoint >= getPositionEncoder()) {
        motor.set(arbUp);
        Logger.getGlobal().log(Level.INFO, name + ": Going Up");
      }

      // if less then move down
      else if (setPoint <= getPositionEncoder()) {
        motor.set(arbDown);
        Logger.getGlobal().log(Level.INFO, name + ": Going Down");
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
      motor.set(0);

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
      return motor.getEncoder().getPosition();
    }

    /**
     * If the elevator is finished moving to a position If we are in the sim blanket return 0 as we
     * cannot move a mechanism
     */
    public boolean isFinished(int position) {

      // if we are in the sim. we're always done
      if (Robot.isSimulation()) {
        return true;
      }

      // If we are at our desired level then we are done
      if (levels.get(position) - (motor.getEncoder().getPosition()) == 0) {
        return true;
      } else {
        return false;
      }
    }
  }

  // Arm
  public static class ArmSubsystemBase extends SubsystemBase {
    public final SparkFlex motor;
    public final SparkFlexConfig motorConfig;

    // The visulizer version of this arm
    private MechanismLigament2d arm;

    private double wantedPos;

    // SparkFlex/max closed loop controller
    SparkClosedLoopController controller;

    private boolean buttonPressed;

    // The PIDF profile to account for variable gravity loads
    private TrapezoidProfile.State state;
    private ArmFeedforward feedforward;

    private double MoveSpeed;
    private String SmartDashboardNickName;

    /** Angle measurements from the encoder to which the arm should goto on command */
    public ArrayList<Double> levels = new ArrayList<>();

    /**
     * Creates an ArmSubsystem Place periodic in robot periodic
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
    public ArmSubsystemBase(
        int ARM_MOTOR_ID,
        IdleMode idleMode,
        Double[] PIDF,
        ArmFeedforward feedforward,
        double MoveSpeed,
        String SmartDashboardNickName) {
      motor = new SparkFlex(ARM_MOTOR_ID, MotorType.kBrushless); // Assign motor controller port

      // Get motor references to motor configs and the sparkFlex
      controller = motor.getClosedLoopController();
      motorConfig = new SparkFlexConfig();

      this.feedforward = feedforward;

      motorConfig.idleMode(idleMode);

      // If we have PIDF set it or disable it
      if (PIDF != null) {
        motorConfig
            .closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pidf(PIDF[0], PIDF[1], PIDF[2], PIDF[3], ClosedLoopSlot.kSlot0);
      } else {
        motorConfig
            .closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pidf(0, 0, 0, 0, ClosedLoopSlot.kSlot0);
      }

      motor.configure(
          motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
      // encoder = new Encoder(1, 1); // Assign encoder ports

      // Set the inital position to hold
      wantedPos = motor.getEncoder().getPosition();
      state = new State(wantedPos, 0);

      this.MoveSpeed = MoveSpeed;
      this.SmartDashboardNickName = SmartDashboardNickName;
    }

    /**
     * Does the setup for the visulizer interpretation of this arm Doesn't put the data to
     * SmartDashboard (Do that when your done adding ligaments)
     *
     * @param root The root object to attach this arm too
     * @param length length of this arm
     * @param angle The angle this is arm is at relative to the root obj (0 is pointing towards the
     *     front, 90 is pointing up)
     */
    public MechanismLigament2d setupVisulizer(MechanismRoot2d root, double length, double angle) {
      arm = root.append(new MechanismLigament2d("elevator", length, angle));
      return arm;
    }

    /**
     * Does the setup for the visulizer interpretation of this arm Doesn't put the data to
     * SmartDashboard (Do that when your done adding ligaments)
     *
     * @param root The parent obj to attach this arm too
     * @param length length of this arm
     * @param angle The angle this is arm is at relative to the parent obj
     */
    public MechanismLigament2d setupVisulizer(
        MechanismLigament2d root, double length, double angle) {
      arm = root.append(new MechanismLigament2d("elevator", length, angle));
      return arm;
    }

    /**
     * Attaches a Mechanism to this arms visulization
     *
     * @param Mechanism to be attached
     * @return Arm with attached ligament
     */
    public MechanismLigament2d AttachMechanism(MechanismLigament2d Mechanism) {
      return arm.append(Mechanism);
    }

    /** Updates the Speed and Encoder position/angle of the Arm to SmartDashboard */
    public void periodic() {
      SmartDashboard.putNumber(SmartDashboardNickName + "Encoder:", getPositionEncoder());

      // If we have an arm visulizer obj then set it to the angle position if IRL
      if (arm != null) {
        if (!Robot.isSimulation()) {
          arm.setAngle(new Rotation2d(getPositionEncoder()));
        }
      }

      // If we are not actively tilting then hold position
      if (!buttonPressed) {
        wantedPos = getPositionEncoder();
        state = new State(wantedPos, 0);
        holdUp(state);
      }
    }

    /** Free hand tilt down. Just hold a button and go. Moves at MoveSpeed */
    public void tiltDown() {

      // mark that we are moving to release PIDF control while moving
      buttonPressed = true;

      Logger.getGlobal().log(Level.INFO, SmartDashboardNickName + ": DOWN " + getPositionEncoder());

      // invert motor and reconfig
      motorConfig.inverted(true);
      motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

      // start moving
      motor.set(MoveSpeed);

      // if in the sim nothing will happen so simulate movement
      if (Robot.isSimulation()) {
        arm.setAngle(arm.getAngle() - MoveSpeed);
      }
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
    public void moveToPosition(double setPoint) {

      // if we are under the setpoint move up
      if (setPoint - 0.5 < getPositionEncoder()) {
        motorConfig.inverted(false);
        motor.set(MoveSpeed);

        // if we are over the setpoint move down
      } else if (setPoint + 0.5 > getPositionEncoder()) {
        motorConfig.inverted(true);
        motor.set(MoveSpeed);

        // if within a degree of the setpoint stop and hold
      } else {
        wantedPos = getPositionEncoder();
        state = new State(wantedPos, 0);
        holdUp(state);
      }
    }

    /** Free hand tilt up. Just hold a button and go. Moves at MoveSpeed */
    public void tiltUp() {

      // mark that we are moving to release PIDF control while moving
      buttonPressed = true;

      Logger.getGlobal().log(Level.INFO, SmartDashboardNickName + ": UP " + getPositionEncoder());

      // uninvert and reconfig
      motorConfig.inverted(false);
      motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

      // start moving
      motor.set(MoveSpeed);

      // if in the sim nothing will happen so simulate movement
      if (Robot.isSimulation()) {
        arm.setAngle(arm.getAngle() + MoveSpeed);
      }
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

      // index the level setpoint from the list
      double setPoint = levels.get(levelNeeded);

      // do the math for the setpoint
      double ff = feedforward.calculate(setPoint * 2 * Math.PI, 0);

      // and set that position for PIDF to aim for
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

      // if we are finished moving to the selected position return true
      return (position - (motor.getEncoder().getPosition()) == 0);
    }

    /**
     * Uses the ff calculator to keep the arm in a single place
     *
     * @param TrapezoidProfile.State Setpoint position to hold
     */
    public void holdUp(TrapezoidProfile.State setpoint) {

      // do the math for the setpoint
      double ff = feedforward.calculate(setpoint.position * 2 * Math.PI, setpoint.velocity);

      // send the math to PIDF to follow
      controller.setReference(0, ControlType.kPosition, ClosedLoopSlot.kSlot0, ff);
    }

    /** Stops the Tilting and also holds using PIDF/{@link #holdUp(TrapezoidProfile.State)} */
    public void stopTilt() {
      motor.set(0);

      // get our current state and tell PIDF to hold it along with releasing the button press
      // I feel like this is redundent as periodic does the same thing
      State setPoint = new State(getPositionEncoder(), 0);
      holdUp(setPoint);
      buttonPressed = false;
    }
  }

  // Game object end effector
  public static class endEffectorSubsystemBase extends SubsystemBase {

    public final SparkFlex motor;
    public final SparkFlexConfig motorConfig;

    private double IntakeSpeed, ShootSpeed;

    private DigitalInput limitSwitch;

    private String name; // Nickname to be used in Smartdashboard messages

    /**
     * Creates a subsystem to control an end effector Designed for end effectors that have
     * mechanisms that directly grab the object and can shoot it Place periodic in robot periodic
     *
     * @param name nickname for this end effector
     * @param MOTOR_ID CANID of motor to control
     * @param CurrentLimit Smart current limit of motor. A good value would be (40-50)
     * @param inverted If the motor is inverted
     * @param PIDF A double list containing PIDF values
     * @param IntakeSpeed The speed to intake at
     * @param ShootSpeed The speed to shoot at
     * @param DIOLimitSwitch Digital input limit switch
     */
    public endEffectorSubsystemBase(
        String name,
        int MOTOR_ID,
        int CurrentLimit,
        boolean inverted,
        double[] PIDF,
        double IntakeSpeed,
        double ShootSpeed,
        DigitalInput DIOLimitSwitch) {
      motor = new SparkFlex(MOTOR_ID, MotorType.kBrushless); // assign motor to port

      // get reference to sparkFlex/max
      motorConfig = new SparkFlexConfig();

      motorConfig.smartCurrentLimit(CurrentLimit);
      motorConfig.inverted(inverted);

      this.name = name;

      // if we have PIDF then set the motor config with it
      if (PIDF != null) {
        motorConfig
            .closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pidf(PIDF[0], PIDF[1], PIDF[2], PIDF[3], ClosedLoopSlot.kSlot0);

        // or else just zero out PIDF to disable it
      } else {
        motorConfig
            .closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pidf(0, 0, 0, 0, ClosedLoopSlot.kSlot0);
      }

      motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

      this.IntakeSpeed = IntakeSpeed;
      this.ShootSpeed = ShootSpeed;
      limitSwitch = DIOLimitSwitch;
    }

    /**
     * Creates a subsystem to control an end effector Designed for end effectors that have
     * mechanisms that directly grab the object and can shoot it Place periodic in robot periodic
     *
     * @param name nickname for this end effector
     * @param MOTOR_ID CANID of motor to control
     * @param CurrentLimit Smart current limit of motor. A good value would be (40-50)
     * @param inverted If the motor is inverted
     * @param PIDF A double list containing PIDF values
     * @param IntakeSpeed The speed to intake at
     * @param ShootSpeed The speed to shoot at
     */
    public endEffectorSubsystemBase(
        String name,
        int MOTOR_ID,
        int CurrentLimit,
        boolean inverted,
        double[] PIDF,
        double IntakeSpeed,
        double ShootSpeed) {
      motor = new SparkFlex(MOTOR_ID, MotorType.kBrushless); // assign motor to port

      // get reference to sparkFlex/max
      motorConfig = new SparkFlexConfig();

      motorConfig.smartCurrentLimit(CurrentLimit);
      motorConfig.inverted(inverted);

      this.name = name;

      // if we have PIDF then set the motor config with it
      if (PIDF != null) {
        motorConfig
            .closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pidf(PIDF[0], PIDF[1], PIDF[2], PIDF[3], ClosedLoopSlot.kSlot0);

        // or else just zero out PIDF to disable it
      } else {
        motorConfig
            .closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pidf(0, 0, 0, 0, ClosedLoopSlot.kSlot0);
      }

      motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

      this.IntakeSpeed = IntakeSpeed;
      this.ShootSpeed = ShootSpeed;
    }

    public void periodic() {
      SmartDashboard.putNumber(name + "-Speed", motor.get());
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
    public double[] calculateLaunchVelocity(
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
                name + ": Cannot shoot to (Unable to break laws of physics): " + target.toString());
        return null; // Not physically possible
      }

      double v = Math.sqrt(vSquared);
      if (v > maxLaunchSpeed) {
        Logger.getGlobal()
            .log(
                Level.WARNING, name + ": Cannot shoot to (Not enough power): " + target.toString());
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
    public double[] calculateLaunchVelocity(
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
                name
                    + ": Cannot find valid solution to shoot (tried the following angles): "
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

      // if we have a limit switch
      if (limitSwitch != null) {

        // intake until limit switch changes
        if (!limitSwitch.get()) {
          motor.set(IntakeSpeed);
        } else {
          motor.set(0);
        }

        // if we don't have a limit switch just move
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
