package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Constants.ReefScorePositions;
import frc.robot.Robot;
import org.mockito.MockedStatic;

public class TestUtils {
  public static void setupRobotMocks(
      MockedStatic<Robot> robotStatic, SendableChooser<ReefScorePositions> mockChooser) {
    // Setup Robot's static accessor method with the mock chooser
    robotStatic.when(Robot::getDesiredScoreSendableChooser).thenReturn(mockChooser);
  }

  public static ReefScorePositions createMockReefPosition(Pose2d pose) {
    return new ReefScorePositions(pose);
  }
}
