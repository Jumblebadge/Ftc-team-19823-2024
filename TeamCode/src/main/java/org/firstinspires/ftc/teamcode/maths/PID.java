package org.firstinspires.ftc.teamcode.maths;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class PID {
    //PID controller class

    private ConstantsForPID constants;
    private double integralSum, out, lastOut, lastError, lastReference;
    private final ElapsedTime timer = new ElapsedTime();

    public PID(ConstantsForPID constants) {
        this.constants = constants;
    }

    public PID(double Kp, double Kd, double Ki, double Kf, double Kl, double pointTunedAt) {
        constants = new ConstantsForPID(Kp, Kd, Ki, Kf, Kl, pointTunedAt);
    }

    public PID(double Kp, double Kd, double Ki, double Kf, double Kl) {
        constants = new ConstantsForPID(Kp, Kd, Ki, Kf, Kl, 0);
    }

    //calculate
    public double pidOut(double reference, double state) {

        if (!Maths.epsilonEquals(lastReference, reference, 0.01)) integralSum = 0;

        double error = reference - state;

        //integral and derivative values
        double derivative = (error - lastError) / timer.seconds();
        if (Math.abs(error) < 2) derivative = 0;
        integralSum += (error * timer.seconds());
        integralSum = Range.clip(integralSum, -constants.Kl(), constants.Kl());
        //weight each term so that tuning makes a difference
        out = (constants.Kp() * error) + (constants.Kd() * derivative) + (constants.Ki() * integralSum) + (constants.Kf() * Math.signum(error));
        out /= 10;
        lastError = error;
        lastReference = reference;
        timer.reset();

        return out;
    }

    public double pidSlideOut(double reference, double state) {

        if (!Maths.epsilonEquals(lastReference, reference, 0.01)) integralSum = 0;

        double error = reference - state;

        //integral and derivative values
        double derivative = (error - lastError) / timer.seconds();
        if (Math.abs(error) < 2) derivative = 0;
        integralSum += (error * timer.seconds());
        integralSum = Range.clip(integralSum, -constants.Kl(), constants.Kl());
        //weight each term so that tuning makes a difference
        out = (constants.Kp() * error) + (constants.Kd() * derivative) + (constants.Ki() * integralSum) + (constants.Kf());
        out /= 10;
        lastError = error;
        lastReference = reference;
        timer.reset();

        return out;
    }

    public double pidSinOut(double reference, double state, double angle) {

        if (!Maths.epsilonEquals(lastReference, reference, 0.01)) integralSum = 0;

        double error = reference - state;

        //integral and derivative values
        double derivative = (error - lastError) / timer.seconds();
        if (Math.abs(error) < 2) derivative = 0;
        integralSum += (error * timer.seconds());
        integralSum = Range.clip(integralSum, -constants.Kl(), constants.Kl());
        //weight each term so that tuning makes a difference
        out = (constants.Kp() * error) + (constants.Kd() * derivative) + (constants.Ki() * integralSum) + (constants.Kf() * Math.sin(Math.toRadians(angle)) + constants.Kf() * 0.1 * Math.cos(Math.toRadians(angle)));
        out /= 10;
        lastError = error;
        lastReference = reference;
        timer.reset();

        return out;
    }

    public double pidAngleOut(double reference, double state) {

        if (!Maths.epsilonEquals(lastReference, reference, 0.01)) integralSum = 0;

        double error = Maths.angleWrapDegrees(reference - state);
        //integral and derivative values
        double derivative = (error - lastError) / timer.seconds();
        if (Math.abs(error) < 2) derivative = 0;
        integralSum += (error * timer.seconds());
        integralSum = Range.clip(integralSum, -constants.Kl(), constants.Kl());
        //weight each term so that tuning makes a difference
        out = (constants.Kp() * error) + (constants.Kd() * derivative) + (constants.Ki() * integralSum) + (constants.Kf() * Math.signum(error));
        out /= 10;

        lastError = error;
        lastOut = out;
        lastReference = reference;
        timer.reset();

        return out;
    }

    public double pidPivotOut(double reference, double state) {

        if (!Maths.epsilonEquals(lastReference, reference, 0.01)) integralSum = 0;

        double error = Maths.angleWrapDegrees(reference - state);
        //integral and derivative values
        double derivative = (error - lastError) / timer.seconds();
        if (Math.abs(error) < 2) derivative = 0;
        integralSum += (error * timer.seconds());
        integralSum = Range.clip(integralSum, -constants.Kl(), constants.Kl());
        //weight each term so that tuning makes a difference
        out = (constants.Kp() * error) + (constants.Kd() * derivative) + (constants.Ki() * integralSum) + (constants.Kf() * Math.cos(Math.toRadians(state) + 0.25));
        out /= 10;

        lastError = error;
        lastOut = out;
        lastReference = reference;
        timer.reset();

        return out;
    }

    public double[] inDepthOutput(double reference, double state) {

        double error = Maths.angleWrapDegrees(reference - state);
        //integral and derivative values
        double derivative = (error - lastError) / timer.seconds();

        return new double[] {out, constants.Kp() * error, constants.Kd() * derivative, constants.Ki() * integralSum, constants.Kf() * Math.signum(error), timer.seconds(), (out - lastOut), error};

    }

    public double ffOut(double reference, double state, double velocityTarget, double accelerationTarget) {
        return pidOut(reference, state + constants.Kv() * velocityTarget + constants.Ka() * accelerationTarget + constants.Ks());
    }

    public double ffAngleOut(double reference, double state, double velocityTarget, double accelerationTarget) {
        return pidAngleOut(reference, state + constants.Kv() * velocityTarget + constants.Ka() * accelerationTarget + constants.Ks());
    }

    public double ffPivotOut(double reference, double state, double velocityTarget, double accelerationTarget) {
        return pidPivotOut(reference, state + constants.Kv() * velocityTarget + constants.Ka() * accelerationTarget + constants.Ks());
    }

    public void setPIDgains(double Kp, double Kd, double Ki, double Kf, double Kl) {
        constants.setPIDgains(Kp, Kd, Ki, Kf, Kl);
    }

    public void setPIDgains(ConstantsForPID constants) {
        this.constants = constants;
    }

    public void setFFgains(double Kv, double Ka, double Ks) {
        constants.setFFgains(Kv, Ka, Ks);
    }

    public void toDefault() {
        constants.toDefault();
    }

}
