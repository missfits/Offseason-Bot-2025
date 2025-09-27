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

        
        public static final double ROBOT_SIZE_X = 10; // need to measure ceridwen to find this value, 10 meters (def wrong) is just for now


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

        public static final double ROTATION_kP = 5;
        public static final double ROTATION_kI = 0;
        public static final double ROTATION_kD = 0;
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


        public static final double OUTTAKE_MOTOR_SPEED = -8.0; 
        public static final double SLOW_OUTTAKE_MOTOR_SPEED = -2.0;
    }
  
    public static class VisionConstants {
        public static final String CAMERA1_NAME = "camera1";  
        public static final String CAMERA2_NAME = "camera2";  


        // all of these values are from dynamene and need to be fine tuned
        public static final double ROBOT_TO_CAM1_X = Units.inchesToMeters(-2); // in meters from center of robot 
        public static final double ROBOT_TO_CAM1_Y = Units.inchesToMeters(-1); // in meters from center of robot 
        public static final double ROBOT_TO_CAM1_Z = Units.inchesToMeters(17); // in meters from the floor?
        
        public static final double ROBOT_TO_CAM2_X = Units.inchesToMeters(13-4.75); // in meters from center of robot 
        public static final double ROBOT_TO_CAM2_Y = Units.inchesToMeters(13-3.125); // in meters from center of robot 
        public static final double ROBOT_TO_CAM2_Z = Units.inchesToMeters(7.5); // in meters from the floor?
        

        // default vision standard deviation
        public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(6, 6, 4);
        public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 0.3);

        public static final double MAX_POSE_AMBIGUITY = 0.2;
        public static final double MAX_VISION_POSE_DISTANCE = 1;
        public static final double MAX_VISION_POSE_Z = 0.1;
        public static final double MAX_VISION_POSE_ROLL = 0.05; // in radians
        public static final double MAX_VISION_POSE_PITCH = 0.05; // in radians
        public static final double VISION_DISTANCE_DISCARD = 10; 

        // all of these constants need to be tuned as well
        public static final double ROBOT_TO_CAM1_ROLL = 0;
        public static final double ROBOT_TO_CAM1_YAW = 0;
        public static final double ROBOT_TO_CAM1_PITCH = 0;

        public static final double ROBOT_TO_CAM2_ROLL = 0;
        public static final double ROBOT_TO_CAM2_YAW = 0;
        public static final double ROBOT_TO_CAM2_PITCH = -Math.PI/9;

        public static final Transform3d ROBOT_TO_CAM1_3D = 
          new Transform3d(new Translation3d(ROBOT_TO_CAM1_X, ROBOT_TO_CAM1_Y, ROBOT_TO_CAM1_Z), new Rotation3d(ROBOT_TO_CAM1_ROLL,ROBOT_TO_CAM1_YAW,ROBOT_TO_CAM1_PITCH)); // in meters from center of robot to 2x4 camera mount
        
        public static final Transform3d ROBOT_TO_CAM2_3D = 
          new Transform3d(new Translation3d(ROBOT_TO_CAM2_X, ROBOT_TO_CAM2_Y, ROBOT_TO_CAM2_Z), new Rotation3d(ROBOT_TO_CAM2_ROLL,ROBOT_TO_CAM2_YAW ,ROBOT_TO_CAM2_PITCH)); // in meters from center of robot to 2x4 camera mount

        public static final double MAX_AVG_DIST_BETWEEN_LAST_EST_POSES = 0.3; // in meters 
        public static final double MAX_AVG_SPEED_BETWEEN_LAST_EST_POSES = MAX_AVG_DIST_BETWEEN_LAST_EST_POSES * 50.;
        public static final int NUM_LAST_EST_POSES = 3;
      
        public static final double VISION_ALIGNMENT_DISCARD = Units.inchesToMeters(1); // in meters
        public static final double INTERMEDIATE_POS_DIST = Units.inchesToMeters(5); // go slower in the last 5 inches
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

