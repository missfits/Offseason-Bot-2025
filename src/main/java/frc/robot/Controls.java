package frc.robot;

import edu.wpi.first.math.MathUtil;
import frc.robot.RobotContainer.JoystickVals;
import frc.robot.Constants.OperatorConstants;

public class Controls {

    public static JoystickVals inputShape(JoystickVals input, boolean transJoystick) {
        return inputShape(input.x(), input.y(), transJoystick);
    }

    public static JoystickVals inputShape(double x, double y, boolean transJoystick) {
        double deadband;
        if (transJoystick) {
            deadband = OperatorConstants.TRANSLATION_JOYSTICK_DEADBAND;
        } else {
            deadband = OperatorConstants.ROTATION_JOYSTICK_DEADBAND;
        }
        return inputShape(x, y, deadband);
    }

    public static JoystickVals inputShape(double x, double y, double deadband) {
        double hypot = Math.hypot(x, y);
        double deadbandedValue = MathUtil.applyDeadband(hypot, deadband);
    
        double scaleFactor;
        if (hypot == 0) { // avoid division by 0 issues
            scaleFactor = 0;
        } else {
            scaleFactor = deadbandedValue * Math.abs(deadbandedValue) / hypot; 
        }

        return new JoystickVals(x * scaleFactor, y * scaleFactor);
    }
}
