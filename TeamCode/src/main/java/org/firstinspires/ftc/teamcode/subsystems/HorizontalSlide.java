package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.utility.DcMotorExW;
import org.firstinspires.ftc.teamcode.utility.RunMotionProfile;

public class HorizontalSlide {

    //TODO fix this
    private final DcMotorExW motor;
    private double target;
    private final TouchSensor touch;
    private final RunMotionProfile profile = new RunMotionProfile(60000,70000,80000,0.1,0,1,0.2, 1);

    public static final double in = -100, mid1 = 200, mid2 = 400, out = 600;
    public static double currentState = in, offset = 0;

    // 0-1100

    public HorizontalSlide(HardwareMap hardwareMap){
        motor = new DcMotorExW(hardwareMap.get(DcMotorEx.class,"Hslide"));
        motor.setPowerThresholds(0.05,0.05);

        touch = hardwareMap.get(TouchSensor.class, "Htouch");

        motor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void moveTo(double target){
        this.target = target;
    }

    public void update() {
        if (touch.isPressed()) {
            resetEncoders();
        }
        motor.setPower(profile.profiledMovement(target, getPosition()));
    }

    public double getError(){
        return target - getPosition();
    }

    public double getPosition() { return motor.getCurrentPosition() + offset; }

    public boolean isTimeDone() { return profile.getProfileDuration() < profile.getCurrentTime(); }

    public boolean isPositionDone() { return Math.abs(getError()) < 10; }

    public void setMotionConstraints(double maxVel, double maxAccel, double maxJerk){
        profile.setMotionConstraints(maxVel, maxAccel, maxJerk);
    }

    public void setPIDcoeffs(double Kp, double Kd, double Ki, double Kf, double limit){
        profile.setPIDcoeffs(Kp, Kd, Ki, Kf, limit);
    }

    public double getMotionTarget(){
        return -profile.getMotionTarget();
    }

    public double getTarget() { return -target; }

    public double getMotionTime() { return profile.getCurrentTime(); }

    public void resetEncoders(){
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        offset = 0;
    }

    public void addOffset(double offset) {
        this.offset = offset;
    }

    public void in(){
        moveTo(in);
        currentState = in;
    }

    public void mid1(){
        moveTo(mid1);
        currentState = mid1;
    }

    public void mid2(){
        moveTo(mid2);
        currentState = mid2;
    }

    public void out(){
        moveTo(out);
        currentState = out;
    }

    public double returnState() {
        return currentState;
    }

}
