package frc.robot.subsystems;

public class Configs {
    public static class AlgaeSubsystem {
        public static final MotorConfig highAlgaeMotorConfig = new MotorConfig(0.5, 0.2, 0.0);
        public static final MotorConfig lowAlgaeMotorConfig = new MotorConfig(0.5, 0.2, 0.0);
    }

    // Example configuration class to hold motor settings
    public static class MotorConfig {
        public final double maxOutput;
        public final double minOutput;
        public final double neutralDeadband;

        public MotorConfig(double maxOutput, double minOutput, double neutralDeadband) {
            this.maxOutput = maxOutput;
            this.minOutput = minOutput;
            this.neutralDeadband = neutralDeadband;
        }
    }
}