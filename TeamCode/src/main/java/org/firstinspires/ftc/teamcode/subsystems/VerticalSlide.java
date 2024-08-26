package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.utility.DcMotorExW;
import org.firstinspires.ftc.teamcode.utility.MotorGroup;
import org.firstinspires.ftc.teamcode.utility.RunMotionProfile;

public class VerticalSlide {

    public final MotorGroup motors;
    private double target;
    private final TouchSensor touch;
    private final DcMotorExW liftLeft, liftRight;
    private final RunMotionProfile profile = new RunMotionProfile(60000,70000,80000,0.1,0,1,0.2, 1);

    public static final double in = 0, mid1 = 600, mid2 = 1200, out = 1900;
    private double currentState = in, offset = 0;

    // 0-1600

    public VerticalSlide(HardwareMap hardwareMap){
        liftLeft = new DcMotorExW(hardwareMap.get(DcMotorEx.class,"Llift"));
        liftRight = new DcMotorExW(hardwareMap.get(DcMotorEx.class,"Rlift"));
        liftRight.setPowerThresholds(0.05,0.05);
        liftLeft.setPowerThresholds(0.05,0.05);

        touch = hardwareMap.get(TouchSensor.class, "Vtouch");

        liftLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        motors = new MotorGroup(liftLeft,liftRight);
    }

    public void moveTo(double target){
        this.target = target;
    }

    public void update() {
        if (touch.isPressed()) {
            resetEncoders();
        }
        motors.setPower(profile.profiledMovement(target, getPosition()),0);
        motors.setPower(profile.profiledMovement(target, getPosition()),1);
    }

    public void disabledPIDsetPower(double power) {
        motors.setPower(power,0);
        motors.setPower(power,1);
    }

    public double getError(){
        return target - getPosition();
    }

    public double getPosition() { return motors.getPosition(0) + offset; }

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
        motors.motors[0].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motors.motors[1].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motors.motors[0].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motors.motors[1].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
