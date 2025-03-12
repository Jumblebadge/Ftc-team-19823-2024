package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannelImpl;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.maths.ConstantsForPID;
import org.firstinspires.ftc.teamcode.utility.wrappers.DcMotorExW;
import org.firstinspires.ftc.teamcode.utility.wrappers.MotorGroup;
import org.firstinspires.ftc.teamcode.utility.RunMotionProfile;

//TODO fix slides
public class PivotingSlide {

    private final MotorGroup slideMotors;
    private final DcMotorExW pivot;
    private double slideTarget = 0, pivotTarget = 0, slideOffset = 0;
    private boolean isPivotManual = false, isReset = false;
    private final DigitalChannelImpl slideLimitSwitch;
    private final AnalogInput pivotEncoder;
    private RunMotionProfile pivotProfile = new RunMotionProfile(20000,5000,5000,new ConstantsForPID(1,0.03,0.4,0,4,0));
    private final RunMotionProfile slideProfile = new RunMotionProfile(200000,80000,80000,new ConstantsForPID(0.3,0,0.4,0.5,4,0));

    public final double MIN = -5, MAX = 420, SET_POINT_1 = MAX / 4, SET_POINT_2 = MAX / 2, SET_POINT_3 = 3 * MAX / 4;
    public enum States {
        MIN,
        SETPOINT_1,
        SETPOINT_2,
        SETPOINT_3,
        MAX
    }
    private States state = States.MIN;

    public PivotingSlide(HardwareMap hardwareMap, boolean slowPivot) {
        DcMotorExW lift1 = new DcMotorExW(hardwareMap.get(DcMotorEx.class, "lift1"));
        DcMotorExW lift2 = new DcMotorExW(hardwareMap.get(DcMotorEx.class, "lift2"));
        DcMotorExW lift3 = new DcMotorExW(hardwareMap.get(DcMotorEx.class, "lift3"));
        pivot = new DcMotorExW(hardwareMap.get(DcMotorEx.class, "pivot"));
        lift1.setPowerThresholds(0.05,0.05);
        lift2.setPowerThresholds(0.05,0.05);
        lift3.setPowerThresholds(0.05,0.05);
        pivot.setPowerThresholds(0.05,0.05);

        pivot.setDirection(DcMotorSimple.Direction.REVERSE);

        slideLimitSwitch = hardwareMap.get(DigitalChannelImpl.class, "slideLimit");

        slideMotors = new MotorGroup(lift1, lift2, lift3);

        resetEncoders();

        pivotEncoder = hardwareMap.get(AnalogInput.class, "pivotEncoder");
    }

    public void moveSlideTo(double target) {
        slideTarget = target;
        if (target > 10) isReset = false;
    }

    public void movePivotTo(double target) { pivotTarget = target; }

    public void update() {
        if (!limitState() && getSlidePosition() < 10 || getSlidePosition() < 0) {
            slideOffset = slideMotors.getPosition(2);
            isReset = true;
        }
        slideMotors.setPowers(slideProfile.profiledSlideMovement(slideTarget, getSlidePosition()));

        if (!isPivotManual) pivot.setPower(pivotProfile.profiledPivotMovement(pivotTarget, getPivotAngle()));
    }

    public void resetEncoders() {
        slideMotors.resetEncoders();
        slideOffset = 0;
    }

    public void setPivotPower(double power) {
        isPivotManual = true;
        pivot.setPower(power);
    }

    public void pivotProfiled() {
        isPivotManual = false;
    }

    public double getSlideError(){
        return slideTarget - getSlidePosition();
    }

    public double getSlidePosition() { return slideMotors.getPosition(2) - slideOffset; }

    public double getPivotAngle() { return  pivotEncoder.getVoltage() * -72 + 244.25; }

    public boolean isTimeDone() { return slideProfile.getProfileDuration() + 0.5 < slideProfile.getCurrentTime(); }

    public boolean isPositionDone() { return Math.abs(getSlideError()) < 22; }

    public void setMotionConstraints(double maxVel, double maxAccel, double maxJerk){
        pivotProfile.setMotionConstraints(maxVel, maxAccel, maxJerk);
    }

    public void setPidConstants(ConstantsForPID constants) {
        pivotProfile.setPidConstants(constants);
    }

    public double getSlideMotionTarget(){
        return pivotProfile.getMotionTarget();
    }

    public double getPivotMotionTarget(){
        return pivotProfile.getMotionTarget();
    }

    public double getSlideTarget() { return slideTarget; }

    public double getMotionTime() { return slideProfile.getCurrentTime(); }

    public void toMin() {
        if (isReset) moveSlideTo(0);
        else moveSlideTo(MIN);
        state = States.MIN;
    }

    public void toSetPoint1() {
        moveSlideTo(SET_POINT_1);
        state = States.SETPOINT_1;
        isReset = false;
    }

    public void toSetPoint2() {
        moveSlideTo(SET_POINT_2);
        state = States.SETPOINT_2;
        isReset = false;
    }

    public void toSetPoint3() {
        moveSlideTo(SET_POINT_3);
        state = States.SETPOINT_3;
        isReset = false;
    }

    public void toMax() {
        moveSlideTo(MAX);
        state = States.MAX;
        isReset = false;
    }

    public States returnState() {
        return state;
    }

    public boolean limitState() {
        return slideLimitSwitch.getState();
    }

}
