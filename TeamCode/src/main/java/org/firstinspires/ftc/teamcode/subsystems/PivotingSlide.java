package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.maths.ConstantsForPID;
import org.firstinspires.ftc.teamcode.utility.DcMotorExW;
import org.firstinspires.ftc.teamcode.utility.MotorGroup;
import org.firstinspires.ftc.teamcode.utility.RunMotionProfile;

public class PivotingSlide {

    public final MotorGroup motors;
    private double slideTarget, pivotTarget, resetOffset;
    private final TouchSensor touch;
    private final DcMotorExW pivot;
    private final DcMotorExW encoder;
    private final RunMotionProfile pivotProfile = new RunMotionProfile(1,1,1,new ConstantsForPID(0,0,0,0,1,0));
    private final RunMotionProfile slideProfile = new RunMotionProfile(60000,70000,80000,new ConstantsForPID(0,0,0,0,1,0));

    public final double min = 0, setPoint1 = 1, setPoint2 = 2, setPoint3 = 3, max = 4;
    public enum PossibleStates {
        MIN,
        SETPOINT_1,
        SETPOINT_2,
        SETPOINT_3,
        MAX
    }
    private PossibleStates state = PossibleStates.MIN;

    public PivotingSlide(HardwareMap hardwareMap) {
        DcMotorExW liftLeft = new DcMotorExW(hardwareMap.get(DcMotorEx.class, "Llift"));
        DcMotorExW liftRight = new DcMotorExW(hardwareMap.get(DcMotorEx.class, "Rlift"));
        pivot = new DcMotorExW(hardwareMap.get(DcMotorEx.class, "pivot"));
        encoder = new DcMotorExW(hardwareMap.get(DcMotorEx.class, "FILL THIS IN"));
        liftRight.setPowerThresholds(0.05,0.05);
        liftLeft.setPowerThresholds(0.05,0.05);
        pivot.setPowerThresholds(0.05, 0.05);

        touch = hardwareMap.get(TouchSensor.class, "touch");

        liftLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        motors = new MotorGroup(liftLeft, liftRight);
    }

    public void moveSlideTo(double target){
        this.slideTarget = target;
    }

    public void movePivotTo(double target) { pivotTarget = target; }

    public void update() {
        if (touch.isPressed()) {
            resetOffset = encoder.getCurrentPosition();
        }
        motors.setPower(slideProfile.profiledMovement(slideTarget, getSlidePosition()),0);
        motors.setPower(slideProfile.profiledMovement(slideTarget, getSlidePosition()),1);
        pivot.setPower(pivotProfile.profiledMovement(pivotTarget, pivot.getCurrentPosition()));
    }

    public void disabledPIDsetPower(double power) {
        motors.setPower(power,0);
        motors.setPower(power,1);
    }

    public double getError(){
        return slideTarget - getSlidePosition();
    }

    public double getSlidePosition() { return encoder.getCurrentPosition() - resetOffset; }

    public double getPivotPosition() { return pivot.getCurrentPosition(); }

    public boolean isTimeDone() { return slideProfile.getProfileDuration() < slideProfile.getCurrentTime(); }

    public boolean isPositionDone() { return Math.abs(getError()) < 10; }

    public void setMotionConstraints(double maxVel, double maxAccel, double maxJerk){
        slideProfile.setMotionConstraints(maxVel, maxAccel, maxJerk);
    }

    public double getMotionTarget(){
        return -slideProfile.getMotionTarget();
    }

    public double getSlideTarget() { return -slideTarget; }

    public double getMotionTime() { return slideProfile.getCurrentTime(); }

    public void toMin() {
        moveSlideTo(min);
        state = PossibleStates.MIN;
    }

    public void toSetPoint1() {
        moveSlideTo(setPoint1);
        state = PossibleStates.SETPOINT_1;
    }

    public void toSetPoint2() {
        moveSlideTo(setPoint2);
        state = PossibleStates.SETPOINT_2;
    }

    public void toSetPoint3() {
        moveSlideTo(setPoint3);
        state = PossibleStates.SETPOINT_3;
    }

    public void toMax() {
        moveSlideTo(max);
        state = PossibleStates.MAX;
    }

    public PossibleStates returnState() {
        return state;
    }

}
