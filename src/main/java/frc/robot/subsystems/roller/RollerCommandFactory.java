package frc.robot.subsystems.roller;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.RollerConstants;

public class RollerCommandFactory {
    private RollerSubsystem m_subsystem;

    public RollerCommandFactory(RollerSubsystem roller) {
        m_subsystem = roller;
    }

    public Command runRoller() {
        return m_subsystem.runRoller(RollerConstants.OUTTAKE_MOTOR_SPEED);
    }

    public Command rollerOff() {
        return m_subsystem.runRollerOff();
    }
}
