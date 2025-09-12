// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.RollerConstants;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.roller.RollerSubsystem;

import frc.robot.Constants.DrivetrainConstants;

public class RobotContainer {

    public record JoystickVals(double x, double y) {}
    
    private final Telemetry logger = new Telemetry(DrivetrainConstants.MAX_TRANSLATION_SPEED);

    private final CommandXboxController joystick = new CommandXboxController(0);
    
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final RollerSubsystem m_roller = new RollerSubsystem();

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
        
        joystick.rightBumper().whileTrue(
            drivetrain.applyRequest(()-> drivetrain.defaultDrive(
                new JoystickVals(joystick.getLeftX(), joystick.getLeftY()), 
                new JoystickVals(joystick.getRightX(), joystick.getRightY()),
                true)
            )
        );

        joystick.x().whileTrue(drivetrain.pointWheelsInX());

        joystick.b().whileTrue(
            drivetrain.applyRequest(() -> drivetrain.point(
                new JoystickVals(joystick.getLeftX(), joystick.getLeftY()))
            )
        );

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
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}