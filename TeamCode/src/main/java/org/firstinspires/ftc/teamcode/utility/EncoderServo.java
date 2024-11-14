package org.firstinspires.ftc.teamcode.utility;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServoImplEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.maths.PID;
import org.firstinspires.ftc.teamcode.maths.SlewRateLimiter;

public class EncoderServo {

    private final CRServoImplEx servo;
    private final AnalogInput encoder;
    private final SlewRateLimiter slew = new SlewRateLimiter();
    private final org.firstinspires.ftc.teamcode.maths.PID PID = new PID(0.05,0.00002,2,0,0.1);
    private double target, r = 5;

    public EncoderServo(CRServoImplEx servo, AnalogInput encoder){
        this.servo = servo;
        this.encoder = encoder;
    }

    public void setPIDcoeffs(double Kp, double Kd, double Ki, double Kf, double limit) {
        PID.setPIDgains(Kp, Kd, Ki, Kf, limit);
    }

    public double getPosition() {
        return -AngleUnit.normalizeDegrees(encoder.getVoltage() / 3.3 * 360);
    }

    public double getError(){
        return AngleUnit.normalizeDegrees(target - getPosition());
    }

    public void PWMrelease() {
        servo.setPwmDisable();
    }

    public void setPosition(double position) {
        target = position;
    }

    public void update(boolean reversed) {
        double PIDout = PID.pidAngleOut(target, getPosition());
        double limited = slew.rateLimit((reversed) ? -PIDout : PIDout,r);
        servo.setPower(limited);
    }

    public void setR(double r){
        this.r = r;
    }

}
