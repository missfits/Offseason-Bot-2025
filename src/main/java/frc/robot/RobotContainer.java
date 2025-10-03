// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
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

import frc.robot.Constants.DrivetrainConstants;

import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.VisionConstants;

import frc.robot.commands.AutoRotateandAlignCommand;
import frc.robot.commands.AutoRotateandAlignCommand.ReefPosition;


public class RobotContainer {
    public record JoystickVals(double x, double y) {}
    
    private final Telemetry logger = new Telemetry(DrivetrainConstants.MAX_TRANSLATION_SPEED);

    private final CommandXboxController joystick = new CommandXboxController(0);
    private final SendableChooser<Command> m_autoChooser; // sendable chooser that holds the autos

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final RollerSubsystem m_roller = new RollerSubsystem();

    public final RollerCommandFactory m_rollerCommandFactory = new RollerCommandFactory(m_roller);

    private final VisionSubsystem m_vision = new VisionSubsystem();
    private final Field2d m_actualField = new Field2d();
    private Pose2d m_ppTargetPose;


    public RobotContainer() {
      configureBindings();
      createNamedCommands();
      m_autoChooser = AutoBuilder.buildAutoChooser();
      SmartDashboard.putData("Auto Chooser", m_autoChooser);
      // Starts recording to data log
      DataLogManager.start();
      // Record both DS control and joystick data
      DriverStation.startDataLog(DataLogManager.getLog());
      // turn off unplugged joystick errors
      DriverStation.silenceJoystickConnectionWarning(true); 

      PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
        m_ppTargetPose = pose;
        SmartDashboard.putNumber("ppTargetPose/x", m_ppTargetPose.getX());
        SmartDashboard.putNumber("ppTargetPose/y", m_ppTargetPose.getY());
        SmartDashboard.putNumber("ppTargetPose/rotation", m_ppTargetPose.getRotation().getRadians());
      });

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
        
        //bindings for roller subsystem
        joystick.leftTrigger().and(joystick.a().negate()).whileTrue(m_rollerCommandFactory.runRoller());
        m_roller.setDefaultCommand(m_rollerCommandFactory.runRollerBack());
        
        // drive in slowmode
        joystick.rightTrigger().and(joystick.a().negate()).whileTrue(
            drivetrain.applyRequest(()-> drivetrain.defaultDrive(
                new JoystickVals(joystick.getLeftX(), joystick.getLeftY()), 
                new JoystickVals(joystick.getRightX(), joystick.getRightY()),
                true)
            )
        );
        
        joystick.leftBumper().whileTrue(new AutoRotateandAlignCommand(drivetrain, ReefPosition.LEFT)); 
        joystick.rightBumper().whileTrue(new AutoRotateandAlignCommand(drivetrain, ReefPosition.RIGHT)); 
        
        // point wheels in x configuration
        joystick.x().whileTrue(drivetrain.pointWheelsInX());
        // snap to left coral station angle
        joystick.y().and(joystick.a().negate()).whileTrue(
          drivetrain.applyRequest(() -> drivetrain.snapToAngle(joystick, FieldConstants.LEFT_CORAL_STATION_ANGLE))
        );
        // snap to right coral station angle
        joystick.b().whileTrue(
          drivetrain.applyRequest(() -> drivetrain.snapToAngle(joystick, FieldConstants.RIGHT_CORAL_STATION_ANGLE))
        );
        // reset the field-centric heading (robot forward direction)
        joystick.a().and(joystick.y()).whileTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        
        m_roller.setDefaultCommand(m_roller.runRollerOff());
        
        drivetrain.registerTelemetry(logger::telemeterize);
        drivetrain.setHeadingController();

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        drivetrain.registerTelemetry(logger::telemeterize);
        // CameraServer.startAutomaticCapture();
    }
    
    private void createNamedCommands() {
        NamedCommands.registerCommand("scoreCoral", m_roller.runRoller(RollerConstants.OUTTAKE_MOTOR_VELOCITY).withTimeout(3));
    }

    // This method loads the auto when it is called, however, it is recommended
    // to first load your paths/autos when code starts, then return the
    // pre-loaded auto/path
    public Command getAutonomousCommand() {
        return m_autoChooser.getSelected();
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
    SmartDashboard.putData(m_actualField);
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