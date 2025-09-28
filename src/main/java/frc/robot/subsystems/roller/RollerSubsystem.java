package frc.robot.subsystems.roller;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;


public class RollerSubsystem extends SubsystemBase {
    private final RollerIOHardware m_IO = new RollerIOHardware();
    
    // constructor
    public RollerSubsystem() {
        m_IO.resetPosition();
    }

    public Command runRollerOff() {
        return new RunCommand(
            () -> {m_IO.setVelocityVoltage(0); SmartDashboard.putString("roller/currentlyRunningCommand", "runRollerOffInstant");},
            this
        ).withTimeout(0).withName("rollerOff");
    }

    public Command runRoller(double velocity) {
        return new RunCommand(
            () -> {m_IO.setVelocityVoltage(velocity); SmartDashboard.putString("roller/currentlyRunningCommand", "runRollerInstant");},
            this
        ).withName("rollerOff");
    }

    public Command setVelocityVoltageToZeroCommand() {
        return new RunCommand(() -> m_IO.setVelocityVoltage(0), this).ignoringDisable(true);
    }

    @Override
    public void periodic() {
        SmartDashboard.putData("roller/subsystem", this);
        SmartDashboard.putNumber("roller/current", m_IO.getCurrent());
        SmartDashboard.putNumber("roller/velocity", m_IO.getVelocity());
    }
 }

 