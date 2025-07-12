package frc.robot.commands;

import static org.mockito.Mockito.*;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.mockito.ArgumentCaptor;

public class SearchForTagTest {
    private SwerveSubsystem mockSwerveDrive;
    private NetworkTableInstance mockNTInstance;
    private NetworkTable mockLimelightTable;
    private NetworkTableEntry mockTagEntry;
    private SearchForTag searchCommand;

    @BeforeEach
    void setUp() {
        // Create mocks
        mockSwerveDrive = mock(SwerveSubsystem.class);
        mockNTInstance = mock(NetworkTableInstance.class);
        mockLimelightTable = mock(NetworkTable.class);
        mockTagEntry = mock(NetworkTableEntry.class);

        // Setup NetworkTables mocking
        when(NetworkTableInstance.getDefault()).thenReturn(mockNTInstance);
        when(mockNTInstance.getTable("limelight")).thenReturn(mockLimelightTable);
        when(mockLimelightTable.getEntry("aprilTags")).thenReturn(mockTagEntry);

        // Create command
        searchCommand = new SearchForTag(mockSwerveDrive);
    }

    @Test
    void testInitialize() {
        searchCommand.initialize();
    }

    @Test
    void testExecute_NoTag() {
        // Setup: No recent tag detection
        when(mockTagEntry.getLastChange()).thenReturn(0.0);
        when(Timer.getFPGATimestamp()).thenReturn(1.0);

        searchCommand.execute();

        // Verify clockwise rotation command
        ArgumentCaptor<ChassisSpeeds> speedsCaptor = ArgumentCaptor.forClass(ChassisSpeeds.class);
        verify(mockSwerveDrive).drive(speedsCaptor.capture());
        
        ChassisSpeeds capturedSpeeds = speedsCaptor.getValue();
        assert(capturedSpeeds.omegaRadiansPerSecond == 0.3);
        assert(capturedSpeeds.vxMetersPerSecond == 0);
        assert(capturedSpeeds.vyMetersPerSecond == 0);
    }

    @Test
    void testExecute_TagFound() {
        // Setup: Recent tag detection
        when(mockTagEntry.getLastChange()).thenReturn(Timer.getFPGATimestamp() * 1e6);

        searchCommand.execute();

        // Verify stop command
        ArgumentCaptor<ChassisSpeeds> speedsCaptor = ArgumentCaptor.forClass(ChassisSpeeds.class);
        verify(mockSwerveDrive).drive(speedsCaptor.capture());
        
        ChassisSpeeds capturedSpeeds = speedsCaptor.getValue();
        assert(capturedSpeeds.omegaRadiansPerSecond == 0);
        assert(capturedSpeeds.vxMetersPerSecond == 0);
        assert(capturedSpeeds.vyMetersPerSecond == 0);
    }

    @Test
    void testEnd() {
        searchCommand.end(false);
        
        // Verify robot stops
        ArgumentCaptor<ChassisSpeeds> speedsCaptor = ArgumentCaptor.forClass(ChassisSpeeds.class);
        verify(mockSwerveDrive).drive(speedsCaptor.capture());
        
        ChassisSpeeds capturedSpeeds = speedsCaptor.getValue();
        assert(capturedSpeeds.omegaRadiansPerSecond == 0);
        assert(capturedSpeeds.vxMetersPerSecond == 0);
        assert(capturedSpeeds.vyMetersPerSecond == 0);
        
    }

    @Test
    void testIsFinished() {
        assert(!searchCommand.isFinished()); // Command should run until canceled
    }
}
