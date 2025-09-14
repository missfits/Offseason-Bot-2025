// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.RollerConstants;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import frc.robot.subsystems.LocalizationCamera;
import frc.robot.subsystems.VisionSubsystem;

import frc.robot.subsystems.roller.RollerSubsystem;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.VisionConstants;

public class RobotContainer {
    public enum RobotName {
        CERIDWEN,
        DYNAMENE
    }
    public static RobotName name = RobotName.CERIDWEN;



    public record JoystickVals(double x, double y) {}
    
    /* Setting up bindings for necessary control of the swerve drive platform */
    // private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
    //         .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
    //         .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    
    private final Telemetry logger = new Telemetry(DrivetrainConstants.MAX_TRANSLATION_SPEED);

    private final CommandXboxController joystick = new CommandXboxController(0);
    

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();


    private final VisionSubsystem m_vision = new VisionSubsystem();
    private final Field2d m_actualField = new Field2d();

    public final RollerSubsystem m_roller = new RollerSubsystem();


    public RobotContainer() {

        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() -> {
                
                SmartDashboard.putNumber("controller/left x", joystick.getLeftX());
                SmartDashboard.putNumber("controller/left y", joystick.getLeftY());
                SmartDashboard.putNumber("controller/right x", joystick.getRightX());
                SmartDashboard.putNumber("controller/right y", joystick.getRightY());
                JoystickVals shapedTrans = Controls.inputShape(joystick.getLeftX(), joystick.getLeftY(), true);
                JoystickVals shapedRot = Controls.inputShape(joystick.getRightX(), joystick.getRightY(), false);
                
                return drive.withVelocityX(-shapedTrans.y() * DrivetrainConstants.MAX_TRANSLATION_SPEED) // Drive forward with negative Y (forward)
                    .withVelocityY(-shapedTrans.x() * DrivetrainConstants.MAX_TRANSLATION_SPEED) // Drive left with negative X (left)
                    .withRotationalRate(-shapedRot.x() * DrivetrainConstants.MAX_ROTATION_SPEED); // Drive counterclockwise with negative X (left)
            
            })
        );
        
        joystick.rightBumper().whileTrue(drivetrain.applyRequest(()-> {
            JoystickVals shapedTrans = Controls.adjustSlowmode(Controls.inputShape(joystick.getLeftX(), joystick.getLeftY(), true));
            JoystickVals shapedRot = Controls.adjustSlowmode(Controls.inputShape(joystick.getRightX(), joystick.getRightY(), false));
            return drive.withVelocityX(-shapedTrans.y() * DrivetrainConstants.MAX_TRANSLATION_SPEED) // Drive forward with negative Y (forward)
                    .withVelocityY(-shapedTrans.x() * DrivetrainConstants.MAX_TRANSLATION_SPEED) // Drive left with negative X (left)
                    .withRotationalRate(-shapedRot.x() * DrivetrainConstants.MAX_ROTATION_SPEED); // Drive counterclockwise with negative X (left)
            
        }));

        joystick.x().whileTrue(drivetrain.pointWheelsInX());

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        drivetrain.setHeadingController();

        //joystick.y().whileTrue(drivetrain.applyRequest(()->drivetrain.snapToAngle(joystick, 0)));
        //joystick.b().whileTrue(drivetrain.applyRequest(()->drivetrain.snapToAngle(joystick, 90)));
        //joystick.a().whileTrue(drivetrain.applyRequest(()->drivetrain.snapToAngle(joystick, 180)));
        //joystick.x().whileTrue(drivetrain.applyRequest(()->drivetrain.snapToAngle(joystick, 270)));



        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        //joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        //bindings for roller subsystem
        joystick.y().whileTrue(m_roller.runRoller(RollerConstants.OUTTAKE_MOTOR_SPEED));
        m_roller.setDefaultCommand(m_roller.runRollerOff());

        drivetrain.registerTelemetry(logger::telemeterize);
        CameraServer.startAutomaticCapture();
    }


    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }

    public void updatePoseEst() {
    List<LocalizationCamera> cameras = m_vision.getLocalizationCameras();

    for (LocalizationCamera cam : cameras){
      updatePoseEst(cam);
    }
    
    m_actualField.setRobotPose(drivetrain.getState().Pose);
  }

    public void updatePoseEst(LocalizationCamera camera){
      EstimatedRobotPose robotPose = camera.getRobotPose();
      
      if (robotPose != null && camera.getTargetFound() && camera.getIsNewResult()) {
        Pose3d estPose3d = robotPose.estimatedPose; // estimated robot pose of vision
        Pose2d estPose2d = estPose3d.toPose2d();

        // check if new estimated pose and previous pose are less than 2 meters apart (fused poseEst)
        double distance = estPose2d.getTranslation().getDistance(drivetrain.getState().Pose.getTranslation());

        SmartDashboard.putNumber("fusedVision/" + camera.getCameraName() + "/distanceBetweenVisionAndActualPose", distance);
        if (distance < VisionConstants.MAX_VISION_POSE_DISTANCE || !camera.isEstPoseJumpy()) {
          drivetrain.setVisionMeasurementStdDevs(camera.getCurrentStdDevs());
          
          // sample drivetrain fusedPose before updating
          Optional<Pose2d> samplePose = drivetrain.samplePoseAt(Utils.fpgaToCurrentTime(robotPose.timestampSeconds));

          if (samplePose.isPresent()){
            SmartDashboard.putNumberArray("fusedVision/" + camera.getCameraName() + "/samplePose",  new double [] {
              samplePose.get().getX(), samplePose.get().getY(), samplePose.get().getRotation().getRadians()});
          }
          
          SmartDashboard.putNumberArray("fusedVision/" + camera.getCameraName() + "/drivetrainBeforeUpdate", new double [] {
          drivetrain.getState().Pose.getX(), drivetrain.getState().Pose.getY(), drivetrain.getState().Pose.getRotation().getRadians()});


          drivetrain.addVisionMeasurement(estPose2d, Utils.fpgaToCurrentTime(robotPose.timestampSeconds));
          camera.updateField(estPose2d);

          // sample drivetrain fusedPose after updating
          SmartDashboard.putNumberArray("fusedVision/" + camera.getCameraName() + "/drivetrainAfterUpdate", new double [] {
            drivetrain.getState().Pose.getX(), drivetrain.getState().Pose.getY(), drivetrain.getState().Pose.getRotation().getRadians()});
            
          SmartDashboard.putNumberArray("fusedVision/" + camera.getCameraName() + "/visionPose2dFiltered" + camera.getCameraName(), new double[] {estPose2d.getX(), estPose2d.getY(), estPose2d.getRotation().getRadians()});
      }

      SmartDashboard.putNumberArray("fusedVision/" + camera.getCameraName() + "/visionPose3D", new double[] {
        estPose3d.getX(),
        estPose3d.getY(),
        estPose3d.getZ(),
        estPose3d.getRotation().toRotation2d().getRadians()
      }); // post vision 3d to smartdashboard
    }
  }

}

