package org.firstinspires.ftc.teamcode.maths;

public class swerveKinematics {

    //swerve kinematics
    public double[] calculate(double forward, double strafe, double rotate, double imu, boolean fieldCentricActive){

        //rotate vectors by imu heading for field centric (if toggled on)
        double strafe1 = -strafe;
        double forward1 = -forward;

        if (fieldCentricActive) {
            strafe1 = Math.cos(Math.toRadians(imu)) * strafe - Math.sin(Math.toRadians(imu)) * forward;
            forward1 = Math.sin(Math.toRadians(imu)) * strafe + Math.cos(Math.toRadians(imu)) * forward;
        }

        //displacement vectors of wheel positions
        double module1X = strafe1 - rotate * 103d / 113d; // top left wheel x
        double module2X = strafe1 - rotate * -103d / 113d; // bottom right wheel y

        double module1Y = forward1 + rotate * -1; //top left wheel x
        double module2Y = forward1 + rotate * 1; // bottom right wheel x

        //extracting the length of our wheel specific vectors (speed)
        double module1Speed = Math.sqrt((module1X * module1X) + (module1Y * module1Y));
        double module2Speed = Math.sqrt((module2X * module2X) + (module2Y * module2Y));

        //make sure that speed values are scaled properly (none go above 1)
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
