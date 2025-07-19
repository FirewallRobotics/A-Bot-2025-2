package frc.robot.util;

import static org.mockito.Mockito.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import org.mockito.MockedStatic;

public class TestUtils {
  /** Sets up basic robot static mocks for testing. */
  public static void setupRobotMocks(MockedStatic<Robot> robotStatic) {
    // Add any needed Robot static method mocks here
  }

  /** Sets up SmartDashboard mocks for testing. */
  public static void setupDashboardMocks(MockedStatic<SmartDashboard> dashboard) {
    // Setup SmartDashboard defaults
    dashboard.when(() -> SmartDashboard.getNumber(anyString(), anyDouble())).thenReturn(0.0);
  }
}
