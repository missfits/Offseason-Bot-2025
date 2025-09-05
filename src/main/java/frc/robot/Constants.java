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
    }
     
    public static class RollerConstants {
        //ALL OF THESE NEED TO BE ADJUSTED
        public static final int ROLLER_MOTOR_ID = 23; // NEED TO CHANGE
        public static final int MOTOR_STATOR_LIMIT = 20; // needs to be tuned

        public static final int COUNTS_PER_REV = 42; // MIGHT NEED TO CHANGE

        public static final double METERS_PER_ROTATION = 1; 
        public static final double MAX_SPEED = 0.0; 
        public static final double SPEED_LOWER_LIMIT = 0.0;
        public static final double SPEED_UPPER_LIMIT = 0.0;

        public static final double OUTTAKE_MOTOR_SPEED = 8.0; 
        public static final double SLOW_OUTTAKE_MOTOR_SPEED = 2.0;
    }
}