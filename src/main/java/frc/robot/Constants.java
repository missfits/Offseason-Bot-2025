package frc.robot;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.generated.TunerConstants;

public class Constants {
    public static class RobotConstants {

    }

    public static class OperatorConstants {
        public static final int DRIVER_JOYSTICK_PORT = 0;
        public static final int COPILOT_JOYSTICK_PORT = 1;

        public static final double TRANSLATION_JOYSTICK_DEADBAND = 0.1;
        public static final double ROTATION_JOYSTICK_DEADBAND = 0.1;

        public static final double SLOWMODE_FACTOR = 0.3;
    }

    public static class DrivetrainConstants {
        public static final double MAX_TRANSLATION_SPEED = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
        public static final double MAX_ROTATION_SPEED = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
        
        public static final double ROTATION_kP = 5;
        public static final double ROTATION_kI = 0;
        public static final double ROTATION_kD = 0;
    }
}
