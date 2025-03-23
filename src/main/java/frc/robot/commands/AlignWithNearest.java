package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.VisionSubsystem;

public class AlignWithNearest extends Command {

  public static String name = frc.robot.Constants.VisionSubsystemConstants.limelightName;
  private final SlewRateLimiter xLimiter, yLimiter, giroLimiter;
  private final PIDController drivePID, strafePID, rotationPID;
  private final double driveOffset, strafeOffset, rotationOffset;

  public static Pose2d[] TagPos = {
    new Pose2d(16.408, 1.048, new Rotation2d(-0.9075712)),
    new Pose2d(16.296, 7.007, new Rotation2d(0.9075712)),
    new Pose2d(11.434, 7.398, new Rotation2d(1.570796)),
    new Pose2d(0, 0, new Rotation2d(0)),
    new Pose2d(0, 0, new Rotation2d(0)),
    new Pose2d(13.787, 2.811, new Rotation2d(2.094395)),
    new Pose2d(14.261, 2.220, new Rotation2d(Math.toRadians(125))),
    new Pose2d(13.840, 5.217, new Rotation2d(-2.111848)),
    new Pose2d(12.365, 5.165, new Rotation2d(-1.012291)),
    new Pose2d(11.638, 4.007, new Rotation2d(0)),
    new Pose2d(12.390, 2.790, new Rotation2d(1.012291)),
    new Pose2d(1.161, 1.048, new Rotation2d(-2.216568)),
    new Pose2d(1.131, 6.950, new Rotation2d(2.181662)),
    new Pose2d(0, 0, new Rotation2d(0)),
    new Pose2d(0, 0, new Rotation2d(0)),
    new Pose2d(6.364, 0.550, new Rotation2d(-1.570796)),
    new Pose2d(3.390, 2.790, new Rotation2d(1.012291)),
    new Pose2d(2.638, 4.007, new Rotation2d(0)),
    new Pose2d(3.365, 5.165, new Rotation2d(-1.012291)),
    new Pose2d(4.840, 5.217, new Rotation2d(-2.111848)),
    new Pose2d(5.538, 3.969, new Rotation2d(3.141593)),
    new Pose2d(4.787, 2.811, new Rotation2d(2.094395))
  };

  public static Pose2d Tag13 = new Pose2d(1.131, 6.950, new Rotation2d(2.181662));
  public static Pose2d Tag12 = new Pose2d(1.161, 1.048, new Rotation2d(-2.216568));
  public static Pose2d Tag2 = new Pose2d(16.296, 7.007, new Rotation2d(0.9075712));
  public static Pose2d Tag1 = new Pose2d(16.408, 1.048, new Rotation2d(-0.9075712));

  public static Pose2d Tag3 = new Pose2d(11.434, 7.398, new Rotation2d(1.570796));
  public static Pose2d Tag16 = new Pose2d(6.364, 0.550, new Rotation2d(-1.570796));

  public static Pose2d Tag17 = new Pose2d(3.390, 2.790, new Rotation2d(1.012291));
  public static Pose2d Tag18 = new Pose2d(2.638, 4.007, new Rotation2d(0));
  public static Pose2d Tag19 = new Pose2d(3.365, 5.165, new Rotation2d(-1.012291));
  public static Pose2d Tag20 = new Pose2d(4.840, 5.217, new Rotation2d(-2.111848));
  public static Pose2d Tag21 = new Pose2d(5.538, 3.969, new Rotation2d(3.141593));
  public static Pose2d Tag22 = new Pose2d(4.787, 2.811, new Rotation2d(2.094395));

  public static Pose2d Tag6 = new Pose2d(13.787, 2.811, new Rotation2d(2.094395));
  public static Pose2d Tag7 = new Pose2d(14.538, 3.969, new Rotation2d(3.141593));
  public static Pose2d Tag8 = new Pose2d(13.840, 5.217, new Rotation2d(-2.111848));
  public static Pose2d Tag9 = new Pose2d(12.365, 5.165, new Rotation2d(-1.012291));
  public static Pose2d Tag10 = new Pose2d(11.638, 4.007, new Rotation2d(0));
  public static Pose2d Tag11 = new Pose2d(12.390, 2.790, new Rotation2d(1.012291));

  public Command targetCommand;

  // add vision as a requirement to run
  public AlignWithNearest() {
        this.xLimiter = new SlewRateLimiter(4);
        this.yLimiter = new SlewRateLimiter(4);
        this.giroLimiter = new SlewRateLimiter(Units.degreesToRadians(720));

        /**
         * PID Controllers for the align
         */
        this.drivePID = new PIDController(
          0.00023,
          0.0000002,
          2);

        this.strafePID = new PIDController(
          0.00023,
          0.0000002,
          2);

        this.rotationPID = new PIDController(
          0.0020645,
          0,
          0);
        /**
         * Boolean for what target to search
         */

        /**
         * Offsets for the limelight
         */
        //this.offsets = limelight.getOffsets(alingToAprilTag);

        this.driveOffset = 2.1;
        this.strafeOffset = -0.2;
        this.rotationOffset = 10.2;
  }

  @Override
  public void initialize() {
    /*
    if (DriverStation.getAlliance().get().equals(Alliance.Red)) {
      targetCommand = RobotContainer.drivebase.driveToPose(new Pose2d(13, 4, new Rotation2d(0)));
    } else {
      targetCommand = RobotContainer.drivebase.driveToPose(new Pose2d(4, 4, new Rotation2d(0)));
    }
    targetCommand.schedule();
    */
  }

  public void execute(){
    double velForward = 0;
        double velStrafe = 0;
        double velGiro = 0;
 
        /**
         * If there is a seen target, calculate the PIDs velocities,
         * otherwise, rotate so the robot can search the target
         */
        if(VisionSubsystem.getTags().length != 0){

            velForward = drivePID.calculate(VisionSubsystem.getTagArea(), driveOffset);
            velStrafe = strafePID.calculate(VisionSubsystem.getXDistance(), strafeOffset);
            velGiro = -rotationPID.calculate(VisionSubsystem.getTagPose2d().getRotation().getDegrees(), rotationOffset);
        } else if(VisionSubsystem.getTags().length == 0){
            velForward = 0;
            velStrafe = 0;
            velGiro = 0.4;
        } else {
            velForward = 0;
            velStrafe = 0;
            velGiro = 0;
        }
 
          // 3. Make the driving smoother
         velForward = xLimiter.calculate(velForward) * 3;
         velStrafe = yLimiter.calculate(velStrafe) * 3;
         velGiro = giroLimiter.calculate(velGiro) * 5;
 
         // 4. Construct desired chassis speeds
         ChassisSpeeds chassisSpeeds;
         
              //Relative to robot
             chassisSpeeds = new ChassisSpeeds(velForward, velStrafe, velGiro);
  }

  @Override
  public void end(boolean inter) {
    targetCommand.cancel();
    RobotContainer.drivebase.drive(new ChassisSpeeds(0, 0, 0));
  }

  @Override
  public boolean isFinished() {
    return RobotContainer.driverXbox.getLeftX() >= 0.3
        || RobotContainer.driverXbox.getLeftY() >= 0.3
        || RobotContainer.driverXbox.getLeftX() <= -0.3
        || RobotContainer.driverXbox.getLeftY() <= -0.3;
  }
}
