// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.RollerConstants;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.roller.RollerCommandFactory;
import frc.robot.subsystems.roller.RollerSubsystem;

import frc.robot.subsystems.LocalizationCamera;
import frc.robot.subsystems.VisionSubsystem;

import frc.robot.subsystems.roller.RollerSubsystem;
import frc.robot.Constants.DrivetrainConstants;

import frc.robot.Constants.VisionConstants;

import frc.robot.commands.AutoRotateandAlignCommand;
import frc.robot.commands.AutoRotateandAlignCommand.ReefPosition;


public class RobotContainer {

    public record JoystickVals(double x, double y) {}
    
    private final Telemetry logger = new Telemetry(DrivetrainConstants.MAX_TRANSLATION_SPEED);

    private final CommandXboxController joystick = new CommandXboxController(0);
    
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final RollerSubsystem m_roller = new RollerSubsystem();

    public final RollerCommandFactory m_rollerCommandFactory = new RollerCommandFactory(m_roller);

    private final VisionSubsystem m_vision = new VisionSubsystem();
    private final Field2d m_actualField = new Field2d();


    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() -> drivetrain.defaultDrive(
                new JoystickVals(joystick.getLeftX(), joystick.getLeftY()), 
                new JoystickVals(joystick.getRightX(), joystick.getRightY()),
                false)
            )
        );
        
        // drive in slowmode
        joystick.rightBumper().whileTrue(
            drivetrain.applyRequest(()-> drivetrain.defaultDrive(
                new JoystickVals(joystick.getLeftX(), joystick.getLeftY()), 
                new JoystickVals(joystick.getRightX(), joystick.getRightY()),
                true)
            )
        );

        // point wheels in x configuration
        joystick.x().whileTrue(drivetrain.pointWheelsInX());
        // reset the field-centric heading (robot forward direction)
        joystick.a().whileTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        // point wheels at left joystick direction
        joystick.b().whileTrue(
            drivetrain.applyRequest(() -> drivetrain.point(
                new JoystickVals(joystick.getLeftX(), joystick.getLeftY()))
            )
        );
        //bindings for roller subsystem
        joystick.y().whileTrue(m_rollerCommandFactory.runRoller());
        m_roller.setDefaultCommand(m_roller.runRollerOff());

        drivetrain.registerTelemetry(logger::telemeterize);
        drivetrain.setHeadingController();

        //joystick.y().whileTrue(drivetrain.applyRequest(()->drivetrain.snapToAngle(joystick, 0)));
        //joystick.b().whileTrue(drivetrain.applyRequest(()->drivetrain.snapToAngle(joystick, 90)));
        //joystick.a().whileTrue(drivetrain.applyRequest(()->drivetrain.snapToAngle(joystick, 180)));
        //joystick.x().whileTrue(drivetrain.applyRequest(()->drivetrain.snapToAngle(joystick, 270)));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        joystick.rightTrigger().whileTrue(new AutoRotateandAlignCommand(drivetrain, ReefPosition.RIGHT)); 
        joystick.leftTrigger().whileTrue(new AutoRotateandAlignCommand(drivetrain, ReefPosition.LEFT)); 
        joystick.y().whileTrue(new AutoRotateandAlignCommand(drivetrain, ReefPosition.CENTER));

        joystick.a().whileTrue(autoScoreCommand(ReefPosition.LEFT));
        joystick.b().whileTrue(autoScoreCommand(ReefPosition.CENTER));
        joystick.x().whileTrue(autoScoreCommand(ReefPosition.RIGHT));

        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);
        CameraServer.startAutomaticCapture();
    }


    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }

     
    public Command autoScoreCommand(ReefPosition side) {
      return new SequentialCommandGroup(
        new AutoRotateandAlignCommand(drivetrain, side)
          .until(drivetrain.isAutoAlignedTrigger()), 
        m_rollerCommandFactory.runRollerWithTimeout());
    }

    public void updatePoseEst() {
    List<LocalizationCamera> cameras = m_vision.getLocalizationCameras();

    for (LocalizationCamera cam : cameras){
      updatePoseEst(cam);
    }
    
    m_actualField.setRobotPose(drivetrain.getState().Pose);
  }

  public void updatePoseEst(LocalizationCamera camera) {
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

        if (samplePose.isPresent()) {
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