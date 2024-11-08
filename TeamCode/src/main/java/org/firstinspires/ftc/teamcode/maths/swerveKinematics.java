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

        //the joystick values (after rotated) converted into vectors (split in x and y) that are wheel specific, displacement vectors per wheel also
        double mod1strafe = strafe1 - rotate * 103d / 113d; // top left wheel x
        double mod2strafe = strafe1 - rotate * -103d / 113d; // bottom right wheel y

        double mod1forward = forward1 + rotate * -1; //top left wheel x
        double mod2forward = forward1 + rotate * 1; // bottom right wheel x

        //extracting the length of our wheel specific vectors (speed)
        double mod1speed = Math.sqrt((mod1strafe * mod1strafe) + (mod1forward * mod1forward));
        double mod2speed = Math.sqrt((mod2strafe * mod2strafe) + (mod2forward * mod2forward));

        //make sure that speed values are scaled properly (none go above 1)
        double max = Math.max(Math.abs(mod1speed), Math.abs(mod2speed));
        if(max > 1) {
            mod1speed /= max;
            mod2speed /= max;
        }

        //extracting the angle of our wheel specific vectors (angle)
        double mod1angle = Math.atan2(mod1strafe, mod1forward) * 180 / Math.PI;
        double mod2angle = Math.atan2(mod2strafe, mod2forward) * 180 / Math.PI;

        return new double[] {mod1speed,mod2speed,mod1angle,mod2angle};
    }
}
