package org.firstinspires.ftc.teamcode.maths;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class PID {
    //PID controller class

    private final ConstantsForPID constants;
    private double integralSum,out,lastError;
    private final ElapsedTime timer = new ElapsedTime();

    public PID(double Kp, double Kd, double Ki, double Kf, double Kl, double pointTunedAt) {
        constants = new ConstantsForPID(Kp, Kd, Ki, Kf, Kl, pointTunedAt);
    }

    public PID(double Kp, double Kd, double Ki, double Kf, double Kl) {
        constants = new ConstantsForPID(Kp, Kd, Ki, Kf, Kl, 0);
    }

    //calculate
    public double pidOut(double error) {
        if (Math.abs(error) > 0) {
            //integral and derivative values
            double derivative = (error - lastError) / timer.seconds();
            integralSum += (error * timer.seconds());
            integralSum = Range.clip(integralSum, -constants.Kl(), constants.Kl());
            //weight each term so that tuning makes a difference
            out = (constants.Kp() * error) + (constants.Kd() * derivative) + (constants.Ki() * integralSum) + (constants.Kf() * Math.signum(error));
            out /= 10;
            lastError = error;
            timer.reset();
        }
        return out;
    }

    public double ffOut(double error, double velocityTarget, double accelerationTarget) {
        return pidOut(error) + constants.Kv() * velocityTarget + constants.Ka() * accelerationTarget + constants.Ks();
    }

    public void setPIDgains(double Kp, double Kd, double Ki, double Kf, double Kl) {
        constants.setPIDgains(Kp, Kd, Ki, Kf, Kl);
    }

    public void setFFgains(double Kv, double Ka, double Ks) {
        constants.setFFgains(Kv, Ka, Ks);
    }

    public void toDefault() {
        constants.toDefault();
    }

}
