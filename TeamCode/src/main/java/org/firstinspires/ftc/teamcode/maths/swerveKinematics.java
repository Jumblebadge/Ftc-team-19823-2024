package org.firstinspires.ftc.teamcode.maths;

import com.acmerobotics.roadrunner.Vector2d;

public class swerveKinematics {

    /**
     * calculates a speed and angle for each module based on the joystick input and current heading of robot
     * @param translateX desired translation in x direction [-1,1]
     * @param translateY driver left stick y [-1,1]
     * @param rotateX driver right stick x [-1,1]
     * @param currentHeading in degrees
     * @param fieldCentricActive is robot in headless mode
     * @return array of doubles in the form of {module1Speed, module2Speed, module1Angle, module2Angle}
     */

    public double[] calculate(double translateX, double translateY, double rotateX, double currentHeading, boolean fieldCentricActive){

        //convert joystick coordinates into polar coordinates
        Vector2d joystickVec = Maths.toPolarCoordinates(new Vector2d(translateX, translateY));
        //square r and retain the sign (allows precise control)
        joystickVec = new Vector2d(joystickVec.x * Math.abs(joystickVec.x), joystickVec.y);

        if (fieldCentricActive) {
            //rotates input vector by heading (i am aware it is scuffed)
            joystickVec = new Vector2d(joystickVec.x, joystickVec.y + currentHeading * Math.PI / 180);
        }

        //back to cartesian for maths!
        joystickVec = Maths.toCartesianCoordinates(joystickVec);
        double strafe = joystickVec.x;
        double forward = joystickVec.y;

        //displacement vectors of wheel positions
        double module1X = strafe - rotateX * 103d / 113d; // top left wheel x
        double module2X = strafe - rotateX * -103d / 113d; // bottom right wheel y

        double module1Y = forward + rotateX * -1; //top left wheel x
        double module2Y = forward + rotateX * 1; // bottom right wheel x

        //extracting the magnitude of our wheel specific vectors (speed)
        double module1Speed = Math.sqrt((module1X * module1X) + (module1Y * module1Y));
        double module2Speed = Math.sqrt((module2X * module2X) + (module2Y * module2Y));

        //make sure that speed values are scaled properly
        double max = Math.max(Math.abs(module1Speed), Math.abs(module2Speed));
        if (max > 1) {
            module1Speed /= max;
            module2Speed /= max;
        }

        //extracting the angle of our wheel specific vectors (angle)
        double module1Angle = Math.atan2(module1X, module1Y) * 180 / Math.PI;
        double module2Angle = Math.atan2(module2X, module2Y) * 180 / Math.PI;

        return new double[] { module1Speed, module2Speed, module1Angle, module2Angle };
    }
}
