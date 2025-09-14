package frc.robot;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
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
        public static final double ROBOT_SIZE_X = 10;

        // copy pasted from robot code 2025 (lines 28 to 43)
        public static final double ROBOT_ROTATION_P = 5; // 11.507 from rotation sys-id @PF 1/13
        public static final double ROBOT_ROTATION_I = 0;
        public static final double ROBOT_ROTATION_D = 0; // 0.10877 from rotation sys-id @PF 1/13
        
        public static final double ROBOT_POSITION_P = 10;
        public static final double ROBOT_POSITION_I = 0;
        public static final double ROBOT_POSITION_D = 0;
    
        public static final double AUTOALIGN_POSITION_P = 4;
        public static final double AUTOALIGN_POSITION_I = 0;
        public static final double AUTOALIGN_POSITION_D = 0;
    
        public static final double TO_TARGET_POSITION_P = 2.5;
        public static final double TO_TARGET_POSITION_I = 0;
        public static final double TO_TARGET_POSITION_D = 0;

    }
    public static class AutoAlignConstants {
        public static final double REEF_OFFSET_RIGHT = 10;
        public static final double REEF_OFFSET_LEFT = 10;
        public static final double INTERMEDIATE_POS_DIST = Units.inchesToMeters(5); // go slower in the last 5 inches
        public static final double kMaxV = 2; // to be tuned
        public static final double kMaxA = 2.5; // to be tuned
    
        public static final double kMaxIntermediateV = 2; // to be tuned
        public static final double kMaxIntermediateA = 1.5; // to be tuned
    }
}
