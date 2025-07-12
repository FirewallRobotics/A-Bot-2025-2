package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;
import org.mockito.MockedStatic;
import org.mockito.Mockito;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.LimelightHelpers.RawFiducial;
import frc.robot.TestBase;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class VisionSubsystemTest extends TestBase {
    // Pipeline indices
    private static final int APRILTAG_PIPELINE_INDEX = 0;
    private static final int REEF_PIPELINE_INDEX = 1;
    private static final int CORAL_PIPELINE_INDEX = 2;
    private static final int ALGAE_PIPELINE_INDEX = 3;

    private VisionSubsystem visionSubsystem;

    @BeforeEach
    void setUp() {
        visionSubsystem = new VisionSubsystem();
    }

    @Test
    void testCanSeeTag_WhenTagVisible() {
        // Mock LimelightHelpers static methods
        try (MockedStatic<LimelightHelpers> mockedStatic = Mockito.mockStatic(LimelightHelpers.class)) {
            LimelightResults mockResults = mock(LimelightResults.class);
            mockResults.valid = true;
            
            // Create mock fiducial target
            LimelightHelpers.LimelightTarget_Fiducial[] mockTargets = 
                new LimelightHelpers.LimelightTarget_Fiducial[1];
            mockTargets[0] = mock(LimelightHelpers.LimelightTarget_Fiducial.class);
            mockTargets[0].fiducialID = 6; // A reef tag ID
            mockResults.targets_Fiducials = mockTargets;

            mockedStatic.when(() -> LimelightHelpers.getLatestResults(anyString()))
                .thenReturn(mockResults);

            assertTrue(VisionSubsystem.CanSeeTag(6));
        }
    }

    @Test
    void testDistanceToReef_WhenReefTagVisible() {
        try (MockedStatic<LimelightHelpers> mockedStatic = Mockito.mockStatic(LimelightHelpers.class)) {
            // Mock setPipelineIndex
            mockedStatic.when(() -> LimelightHelpers.setPipelineIndex(anyString(), anyInt()))
                       .then(invocation -> null);

            // Create a RawFiducial with proper constructor values
            RawFiducial mockFiducial = mock(RawFiducial.class);
            mockFiducial.id = 6;  // reef tag ID
            mockFiducial.distToRobot = 2.5;  // Set distance explicitly

            RawFiducial[] mockFiducials = new RawFiducial[] { mockFiducial };

            mockedStatic.when(() -> LimelightHelpers.getRawFiducials(anyString()))
                       .thenReturn(mockFiducials);

            assertEquals(2.5, VisionSubsystem.DistanceToReef(), 0.01);
            
            // Verify the pipeline was set to AprilTag pipeline
            mockedStatic.verify(() -> LimelightHelpers.setPipelineIndex(anyString(), eq(APRILTAG_PIPELINE_INDEX)));
        }
    }

    @Test
    void testDistanceToReef_WhenNoReefTagVisible() {
        // Mock LimelightHelpers static methods
        try (MockedStatic<LimelightHelpers> mockedStatic = mockStatic(LimelightHelpers.class)) {
            RawFiducial[] mockFiducials = new RawFiducial[1];
            mockFiducials[0] = new RawFiducial(3, 2.5, 0, 0, 0, 0, 0);  // Fix constructor call

            mockedStatic.when(() -> LimelightHelpers.getRawFiducials(anyString()))
                .thenReturn(mockFiducials);

            assertEquals(-1, VisionSubsystem.DistanceToReef(), 0.01);
        }
    }

    @Test
    void testUpdatePositionOnField_WithValidTags() {
        // Mock LimelightHelpers static methods
        try (MockedStatic<LimelightHelpers> mockedStatic = mockStatic(LimelightHelpers.class)) {
            LimelightHelpers.PoseEstimate mockEstimate = mock(LimelightHelpers.PoseEstimate.class);
            mockEstimate.tagCount = 2;
            mockEstimate.pose = new Pose2d();
            mockEstimate.timestampSeconds = 1.0;

            mockedStatic.when(() -> LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(anyString()))
                .thenReturn(mockEstimate);

            visionSubsystem.UpdatePositionOnField();
            assertFalse(visionSubsystem.doRejectUpdate);
        }
    }

    @Test
    void testUpdatePositionOnField_WithNoTags() {
        // Mock LimelightHelpers static methods
        try (MockedStatic<LimelightHelpers> mockedStatic = mockStatic(LimelightHelpers.class)) {
            LimelightHelpers.PoseEstimate mockEstimate = mock(LimelightHelpers.PoseEstimate.class);
            mockEstimate.tagCount = 0;
            mockEstimate.pose = new Pose2d();
            mockEstimate.timestampSeconds = 1.0;

            mockedStatic.when(() -> LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(anyString()))
                .thenReturn(mockEstimate);

            visionSubsystem.UpdatePositionOnField();
            assertTrue(visionSubsystem.doRejectUpdate);
        }
    }

    @Test
    void testGetTagArea() {
        try (MockedStatic<LimelightHelpers> mockedStatic = Mockito.mockStatic(LimelightHelpers.class)) {
            LimelightResults mockResults = mock(LimelightResults.class);
            mockResults.valid = true;
            
            LimelightHelpers.LimelightTarget_Fiducial mockTarget = mock(LimelightHelpers.LimelightTarget_Fiducial.class);
            mockTarget.ta = 5.0;  // Set expected area
            mockResults.targets_Fiducials = new LimelightHelpers.LimelightTarget_Fiducial[] { mockTarget };

            mockedStatic.when(() -> LimelightHelpers.getLatestResults(anyString()))
                .thenReturn(mockResults);

            assertEquals(5.0, VisionSubsystem.getTagArea(), 0.01);
        }
    }

    @Test
    void testDistanceToCoralStation_WhenVisible() {
        try (MockedStatic<LimelightHelpers> mockedStatic = Mockito.mockStatic(LimelightHelpers.class)) {
            RawFiducial mockFiducial = mock(RawFiducial.class);
            mockFiducial.id = 1;  // Coral station tag ID
            mockFiducial.distToRobot = 3.5;

            mockedStatic.when(() -> LimelightHelpers.getRawFiducials(anyString()))
                .thenReturn(new RawFiducial[] { mockFiducial });

            assertEquals(3.5, VisionSubsystem.DistanceToCoralStation(), 0.01);
        }
    }

    @Test
    void testDistanceToProcessor_WhenVisible() {
        try (MockedStatic<LimelightHelpers> mockedStatic = Mockito.mockStatic(LimelightHelpers.class)) {
            RawFiducial mockFiducial = mock(RawFiducial.class);
            mockFiducial.id = 3;  // Processor tag ID
            mockFiducial.distToRobot = 4.0;

            mockedStatic.when(() -> LimelightHelpers.getRawFiducials(anyString()))
                .thenReturn(new RawFiducial[] { mockFiducial });

            assertEquals(4.0, VisionSubsystem.DistanceToProcessor(), 0.01);
        }
    }

    @Test
    void testCanSeeAlgae() {
        try (MockedStatic<LimelightHelpers> mockedStatic = Mockito.mockStatic(LimelightHelpers.class)) {
            mockedStatic.when(() -> LimelightHelpers.getTargetColor(anyString()))
                .thenReturn(new double[] { 1.0, 0.0, 0.0 });  // Non-negative value indicates target found

            assertTrue(VisionSubsystem.CanSeeAlgae());
        }
    }

    @Test
    void testCanSeeAlgae_WhenNotVisible() {
        try (MockedStatic<LimelightHelpers> mockedStatic = Mockito.mockStatic(LimelightHelpers.class)) {
            mockedStatic.when(() -> LimelightHelpers.getTargetColor(anyString()))
                .thenReturn(new double[] { -1.0, -1.0, -1.0 });  // Negative values indicate no target

            assertFalse(VisionSubsystem.CanSeeAlgae());
        }
    }
    
    @Test
    void testGetReefLocationPose3d_WhenVisible() {
        try (MockedStatic<LimelightHelpers> mockedStatic = Mockito.mockStatic(LimelightHelpers.class)) {
            LimelightResults mockResults = mock(LimelightResults.class);
            mockResults.valid = true;
            
            LimelightHelpers.LimelightTarget_Fiducial mockTarget = mock(LimelightHelpers.LimelightTarget_Fiducial.class);
            mockTarget.fiducialID = 6;  // Reef tag ID
            Pose3d expectedPose = new Pose3d();
            when(mockTarget.getTargetPose_RobotSpace()).thenReturn(expectedPose);
            
            mockResults.targets_Fiducials = new LimelightHelpers.LimelightTarget_Fiducial[] { mockTarget };

            mockedStatic.when(() -> LimelightHelpers.getLatestResults(anyString()))
                .thenReturn(mockResults);

            assertEquals(expectedPose, VisionSubsystem.getReefLocationPose3d());
        }
    }

    // TODO: Test multiple tags in view
    // - Test priority/sorting when multiple reef tags are visible
    // - Test behavior when both reef and processor tags are visible
    // - Test distance calculations with multiple tags
    
    // TODO: Test invalid/error cases
    // - Test behavior when LimelightResults is null
    // - Test behavior when fiducials array is empty
    // - Test behavior when network connection fails
    
    // TODO: Test periodic() method
    // - Test SmartDashboard updates
    // - Test pose estimation updates
    // - Test different robot states (moving vs stationary)
    
    // TODO: Test pipeline switching scenarios
    // - Test switching between AprilTag and color pipelines
    // - Test pipeline retention between method calls
    // - Test invalid pipeline index handling
    
    // TODO: Test getBotPoseEstimate edge cases
    // - Test with single tag visible
    // - Test with multiple tags at different distances
    // - Test with tags at extreme angles
    
    // TODO: Test getRobotPoseInFieldSpace
    // - Test simulation vs real hardware differences
    // - Test coordinate transformations
    // - Test field-relative positioning accuracy
}
