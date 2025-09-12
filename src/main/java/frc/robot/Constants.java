package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.robot.RobotContainer.RobotName;
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
    public static class VisionConstants {
    public static final String CAMERA1_NAME = "beam_camera";  
    public static final String CAMERA2_NAME = "swerve_camera";  

    public static final double ROBOT_TO_CAM1_X = RobotContainer.name == RobotName.CERIDWEN ? Units.inchesToMeters(-2) : 0.31115 ; // in meters from center of robot 
    public static final double ROBOT_TO_CAM1_Y = RobotContainer.name == RobotName.CERIDWEN ? Units.inchesToMeters(-1) : -0.0508; // in meters from center of robot 
    public static final double ROBOT_TO_CAM1_Z = RobotContainer.name == RobotName.CERIDWEN ? Units.inchesToMeters(17) : 0.1397; // in meters from the floor?
    
    public static final double ROBOT_TO_CAM2_X = RobotContainer.name == RobotName.CERIDWEN ? Units.inchesToMeters(13-4.75) : 0 ; // in meters from center of robot 
    public static final double ROBOT_TO_CAM2_Y = RobotContainer.name == RobotName.CERIDWEN ? Units.inchesToMeters(13-3.125) : 0; // in meters from center of robot 
    public static final double ROBOT_TO_CAM2_Z = RobotContainer.name == RobotName.CERIDWEN ? Units.inchesToMeters(7.5) : 0; // in meters from the floor?
    

    // default vision standard deviation
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(6, 6, 4);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 0.3);

    public static final double MAX_POSE_AMBIGUITY = 0.2;
    public static final double MAX_VISION_POSE_DISTANCE = 1;
    public static final double MAX_VISION_POSE_Z = 0.1;
    public static final double MAX_VISION_POSE_ROLL = 0.05; // in radians
    public static final double MAX_VISION_POSE_PITCH = 0.05; // in radians
    public static final double VISION_DISTANCE_DISCARD = 10; 

    public static final double VISION_ALIGNMENT_DISCARD = Units.inchesToMeters(1); // in meters


    public static final Translation2d ROBOT_TO_CAM1 = 
      new Translation2d(ROBOT_TO_CAM1_X, ROBOT_TO_CAM1_Y); // in meters from center of robot to 2x4 camera mount

    public static final Translation2d ROBOT_TO_CAM2 = 
      new Translation2d(ROBOT_TO_CAM2_X, ROBOT_TO_CAM2_Y); // in meters from center of robot to 2x4 camera mount

    public static final Transform3d ROBOT_TO_CAM1_3D = 
      new Transform3d(new Translation3d(ROBOT_TO_CAM1_X, ROBOT_TO_CAM1_Y, ROBOT_TO_CAM1_Z), new Rotation3d(0,0,0)); // in meters from center of robot to 2x4 camera mount
    
    public static final Transform3d ROBOT_TO_CAM2_3D = 
      new Transform3d(new Translation3d(ROBOT_TO_CAM2_X, ROBOT_TO_CAM2_Y, ROBOT_TO_CAM2_Z), new Rotation3d(0,0,-Math.PI/9)); // in meters from center of robot to 2x4 camera mount
  

    public static final double CAMERA_HEIGHT = 0.0951738; // in meters from floor to camera center
    public static final double CAMERA_PITCH = 0; // in radians, bogus
    public static final double TARGET_HEIGHT = 0.3048; // in meters to the middle of the apriltag on reef
    public static final double TARGET_PITCH = 0;

    public static final double MAX_AVG_DIST_BETWEEN_LAST_EST_POSES = 0.3; // in meters 
    public static final double MAX_AVG_SPEED_BETWEEN_LAST_EST_POSES = MAX_AVG_DIST_BETWEEN_LAST_EST_POSES * 50.;
    public static final int NUM_LAST_EST_POSES = 3;
  }
}
