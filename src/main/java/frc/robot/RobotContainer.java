// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import frc.robot.Constants.DrivetrainConstants;

public class RobotContainer {

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
            SmartDashboard.putNumber("controller/left x", joystick.getLeftX());
            SmartDashboard.putNumber("controller/left y", joystick.getLeftY());
            SmartDashboard.putNumber("controller/right x", joystick.getRightX());
            SmartDashboard.putNumber("controller/right y", joystick.getRightY());
            JoystickVals shapedTrans = Controls.adjustSlowmode(Controls.inputShape(joystick.getLeftX(), joystick.getLeftY(), true));
            JoystickVals shapedRot = Controls.adjustSlowmode(Controls.inputShape(joystick.getRightX(), joystick.getRightY(), false));
            SmartDashboard.putNumber("controller/slowmode left x", shapedTrans.x());
            SmartDashboard.putNumber("controller/slowmode left y", shapedTrans.y());
            SmartDashboard.putNumber("controller/slowmode right x", shapedRot.x());
            SmartDashboard.putNumber("controller/slowmode right y", shapedRot.y());
            return drive.withVelocityX(-shapedTrans.y() * DrivetrainConstants.MAX_TRANSLATION_SPEED) // Drive forward with negative Y (forward)
                    .withVelocityY(-shapedTrans.x() * DrivetrainConstants.MAX_TRANSLATION_SPEED) // Drive left with negative X (left)
                    .withRotationalRate(-shapedRot.x() * DrivetrainConstants.MAX_ROTATION_SPEED); // Drive counterclockwise with negative X (left)
            
        }));

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
