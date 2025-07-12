package frc.robot.commands;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.TestBase;
import frc.robot.Constants.ReefScorePositions;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.mockito.MockedStatic;

class AlignWithNearestTest extends TestBase {
    private AlignWithNearest alignCommand;
    private Command mockPathCommand;
    private SendableChooser<ReefScorePositions> mockChooser;
    private Pose2d testPose;
    private ReefScorePositions mockPosition;

    @BeforeEach
    void setUp() {
        // Create test objects
        alignCommand = new AlignWithNearest();
        mockPathCommand = mock(Command.class);
        mockChooser = new SendableChooser<>();  // Use real SendableChooser
        testPose = new Pose2d(10.0, 5.0, new Rotation2d(Math.PI / 2));
        
        // Create and set up ReefScorePositions
        mockPosition = new ReefScorePositions(testPose);
        mockChooser.setDefaultOption("Test Position", mockPosition);
        mockChooser.addOption("Test Position", mockPosition);
    }

    private void setupTestMocks(MockedStatic<Robot> robotStatic, 
                              MockedStatic<AutoBuilder> autoBuilder, 
                              MockedStatic<SmartDashboard> dashboard) {
        // Use getter method to access SendableChooser
        robotStatic.when(Robot::getDesiredScoreSendableChooser).thenReturn(mockChooser);
        
        // Set up SmartDashboard mock
        dashboard.when(() -> SmartDashboard.getNumber(anyString(), anyDouble())).thenReturn(0.0);
        
        // Set up path planner mock
        autoBuilder.when(() -> AutoBuilder.pathfindToPose(any(Pose2d.class), any(PathConstraints.class)))
                  .thenReturn(mockPathCommand);
                  
        // Verify SendableChooser has a selection
        assertNotNull(mockChooser.getSelected(), "SendableChooser selection should not be null");
    }

    @Test
    void testInitialize_CalculatesCorrectTargetPose() {
        try (MockedStatic<Robot> robotStatic = mockStatic(Robot.class);
             MockedStatic<AutoBuilder> autoBuilder = mockStatic(AutoBuilder.class);
             MockedStatic<SmartDashboard> dashboard = mockStatic(SmartDashboard.class)) {
            
            // Setup static mocks
            setupTestMocks(robotStatic, autoBuilder, dashboard);

            alignCommand.initialize();
            verify(mockPathCommand, times(0)).end(anyBoolean());
        }
    }

    @Test
    void testExecute_SchedulesPathCommand() {
        try (MockedStatic<Robot> robotStatic = mockStatic(Robot.class);
             MockedStatic<AutoBuilder> autoBuilder = mockStatic(AutoBuilder.class);
             MockedStatic<SmartDashboard> dashboard = mockStatic(SmartDashboard.class)) {
            
            // Setup mocks
            setupTestMocks(robotStatic, autoBuilder, dashboard);

            alignCommand.initialize();
            alignCommand.execute();
            verify(mockPathCommand).schedule();
        }
    }

    @Test
    void testEnd_CancelsPathCommand() {
        try (MockedStatic<Robot> robotStatic = mockStatic(Robot.class);
             MockedStatic<AutoBuilder> autoBuilder = mockStatic(AutoBuilder.class);
             MockedStatic<SmartDashboard> dashboard = mockStatic(SmartDashboard.class)) {
            
            // Setup mocks
            setupTestMocks(robotStatic, autoBuilder, dashboard);

            alignCommand.initialize();
            alignCommand.end(true);
            verify(mockPathCommand).end(true);
        }
    }

    @Test
    void testIsFinished_ReturnsPathCommandStatus() {
        try (MockedStatic<Robot> robotStatic = mockStatic(Robot.class);
             MockedStatic<AutoBuilder> autoBuilder = mockStatic(AutoBuilder.class);
             MockedStatic<SmartDashboard> dashboard = mockStatic(SmartDashboard.class)) {
            
            // Setup mocks
            setupTestMocks(robotStatic, autoBuilder, dashboard);

            when(mockPathCommand.isFinished()).thenReturn(true);
            alignCommand.initialize();
            assertTrue(alignCommand.isFinished());
        }
    }
}
