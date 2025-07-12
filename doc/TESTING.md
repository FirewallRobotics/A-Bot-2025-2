# Robot Code Testing Guide

## Introduction to Testing Robot Code

### Why We Test
- Catch problems before competitions
- Ensure commands work without real hardware
- Make code changes safely
- Document expected behavior

### Testing Tools We Use
1. **JUnit 5** - Main testing framework
   - Runs our tests
   - Provides assertion methods
   - Helps organize test cases

2. **Mockito** - Creates fake objects
   - Simulates hardware
   - Mocks subsystems
   - Verifies behavior

## Writing Your First Test

### Basic Test Structure
```java
@Test
void testExampleBehavior() {
    // 1. Setup - Prepare objects and data
    Command command = new ExampleCommand();

    // 2. Execute - Run the code being tested
    command.initialize();

    // 3. Verify - Check the results
    assertTrue(command.isFinished());
}
```

### Common Testing Patterns

#### Testing Commands
```java
// Command Test Pattern
@Test
void testCommandPattern() {
    // 1. Create command and mock dependencies
    ExampleCommand command = new ExampleCommand();
    Subsystem mockSubsystem = mock(Subsystem.class);

    // 2. Set up expected behavior
    when(mockSubsystem.getValue()).thenReturn(42);

    // 3. Run command methods
    command.initialize();
    command.execute();

    // 4. Verify results
    verify(mockSubsystem).setValue(42);
}
```

#### Testing Hardware Interactions
```java
// Hardware Test Pattern
@Test
void testHardwarePattern() {
    // 1. Mock hardware components
    Motor mockMotor = mock(Motor.class);
    Encoder mockEncoder = mock(Encoder.class);

    // 2. Set up sensor readings
    when(mockEncoder.getPosition()).thenReturn(10.0);

    // 3. Run system under test
    subsystem.periodic();

    // 4. Verify motor commands
    verify(mockMotor).set(0.5);
}
```

## Leveraging GitHub Copilot for Testing

### Best Practices for Test Generation

1. **Start with Clear Test Structure Comments**
```java
// Given: Describe initial setup and conditions
// When: Describe the action being tested
// Then: Describe expected outcomes
@Test
void testExample() {
    // Copilot will suggest complete test implementation
}
```

2. **Use Descriptive Method Names**
```java
// GOOD - Clear and specific
testDriveCommand_WhenSpeedZero_MotorsShouldStop()

// BAD - Too vague
testDrive()
```

### Effective Copilot Prompts

1. **Setting Context**
```java
// TEST: Command that aligns robot with nearest scoring position
// REQUIRES:
// - Robot position from drivetrain
// - Vision system for AprilTag detection
// - Path planning for movement
// BEHAVIORS:
// - Finds nearest scoring position
// - Plans path to position
// - Executes movement
```

2. **Requesting Specific Test Cases**
```java
// Generate tests for edge cases:
// 1. No AprilTags visible
// 2. Multiple valid targets
// 3. Robot already at target
// 4. Path blocked
```

### Testing Patterns with Copilot

1. **State-Based Testing**
```java
// TEST STATES:
// 1. Initial State: Robot at (0,0)
// 2. Action: Move to target
// 3. Final State: Robot at target
// 4. Verify: Position and orientation
```

2. **Behavior-Based Testing**
```java
// TEST INTERACTIONS:
// 1. Mock dependencies
// 2. Set expectations
// 3. Execute command
// 4. Verify method calls
```

## Best Practices

### 1. Test Organization
- Group related tests in same class
- Use descriptive method names
- Follow naming pattern: `test[Method]_[Scenario]`

### 2. Mock Dependencies
- Don't use real hardware in tests
- Mock external systems (NetworkTables, etc)
- Use `@BeforeEach` for common setup

### 3. Test Independence
- Each test should run alone
- Reset state between tests
- Don't share mutable objects

### 4. Verification
- Test both success and failure cases
- Verify all important behaviors
- Check edge cases

## Building Resilient Tests

### 1. Dependency Isolation

```java
// GOOD - Explicitly declare and mock dependencies
class CommandTest {
    @Mock DriveSubsystem drive;
    @Mock VisionSubsystem vision;

    @BeforeEach
    void setup() {
        // Initialize mocks
    }
}

// BAD - Hidden dependencies
class CommandTest {
    void test() {
        // Direct hardware access
        RobotContainer.getDrive()
    }
}
```

### 2. Test Data Management

```java
// GOOD - Clear test data setup
private TestData createTestData() {
    return new TestData.Builder()
        .withPosition(new Pose2d(1, 1, new Rotation2d()))
        .withTarget(AprilTag.builder().id(5).build())
        .build();
}

// BAD - Magic numbers and unclear data
pose = new Pose2d(1.257, 3.142, new Rotation2d());
```

### 3. Error Handling Tests

```java
// Generate comprehensive error case tests
// ERROR CASES:
// 1. Null inputs
// 2. Invalid configurations
// 3. Hardware failures
// 4. Communication timeouts
// 5. Resource conflicts
```

## Maintainable Test Code

### 1. Test Organization

```java
class AlignCommandTest {
    // Group 1: Initialization Tests
    @Nested
    class InitializationTests {
        // Test command setup
    }

    // Group 2: Execution Tests
    @Nested
    class ExecutionTests {
        // Test command running
    }
}
```

### 2. Shared Test Utilities

```java
// Create common testing utilities
class TestUtils {
    static Pose2d createTestPose() { /*...*/ }
    static void setupMockVision() { /*...*/ }
    static void verifyRobotState() { /*...*/ }
}
```

## Common Pitfalls

### 1. Static Methods
```java
// WRONG - Can't mock static method directly
when(Robot.getInstance()).thenReturn(mockRobot);

// RIGHT - Use MockedStatic
try (MockedStatic<Robot> robotStatic = mockStatic(Robot.class)) {
    robotStatic.when(Robot::getInstance).thenReturn(mockRobot);
}
```

### 2. Hardware Dependencies
```java
// WRONG - Uses real hardware
motor.set(0.5);

// RIGHT - Use mocked hardware
when(mockMotor.get()).thenReturn(0.5);
verify(mockMotor).set(0.5);
```

### 3. Test Isolation
```java
// WRONG - Shared state between tests
static Command sharedCommand;

// RIGHT - Fresh objects for each test
@BeforeEach
void setUp() {
    command = new TestCommand();
}
```

## Copilot Prompting Strategies

### 1. Feature-Based Prompts
```java
// Generate tests for AlignCommand feature:
// - Command aligns robot with scoring position
// - Uses vision for position detection
// - Plans and executes path
// Include: Setup, execution, verification
```

### 2. Scenario-Based Prompts
```java
// Generate tests for competition scenarios:
// 1. Normal scoring approach
// 2. Recovery from bump/collision
// 3. Handle vision interference
// 4. Timeout conditions
```

### 3. Maintenance-Focused Prompts
```java
// Refactor tests to improve:
// 1. Remove duplication
// 2. Clarify test intentions
// 3. Improve error messages
// 4. Add documentation
```

## Continuous Testing Strategy

### 1. Test Coverage Goals
- Core functionality: 90%+ coverage
- Error handling: 100% coverage
- Edge cases: Documented and tested
- Integration points: Verified with mocks

### 2. Test Review Checklist
- [ ] Tests follow naming convention
- [ ] Mocks are properly used
- [ ] Error cases are covered
- [ ] Documentation is clear
- [ ] No hardcoded values

## Resources
- [JUnit 5 User Guide](https://junit.org/junit5/docs/current/user-guide/)
- [Mockito Documentation](https://javadoc.io/doc/org.mockito/mockito-core/latest/org/mockito/Mockito.html)
- [WPILib Unit Testing](https://docs.wpilib.org/en/stable/docs/software/basic-programming/unit-testing.html)

## Troubleshooting Guide

### Common Test Failures
1. NullPointerException
   - Check mock setup
   - Verify object initialization
   - Look for missing dependencies

2. MockitoException
   - Verify mock syntax
   - Check static mocking
   - Ensure proper mock cleanup

3. AssertionError
   - Compare expected vs actual values
   - Check timing issues
   - Verify mock behavior setup
