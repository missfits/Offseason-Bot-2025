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

    /**
     * if x and y within deadband, x=0 and y=0
     * square x and y joystick values while maintaining original angle 
    */
    public static JoystickVals inputShape(double x, double y, double deadband) {
        // manipulate hypotenuse length to maintain angle
        double hypot = Math.hypot(x, y);
        // apply deadband
        double deadbandedValue = MathUtil.applyDeadband(hypot, deadband);
    
        double scaleFactor;
        if (hypot == 0) { // avoid division by 0 issues
            scaleFactor = 0;
        } else {
            // use ratio (new length)/(old length) of hypotenuse
            scaleFactor = deadbandedValue * Math.abs(deadbandedValue) / hypot; 
        }

        return new JoystickVals(x * scaleFactor, y * scaleFactor);
    }

    public static JoystickVals adjustSlowmode(JoystickVals input) {
        return new JoystickVals(input.x() * OperatorConstants.SLOWMODE_FACTOR, input.y() * OperatorConstants.SLOWMODE_FACTOR);
    }
    
}
