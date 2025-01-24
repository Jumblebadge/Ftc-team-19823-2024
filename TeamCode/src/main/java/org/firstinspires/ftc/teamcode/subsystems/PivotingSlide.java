package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.maths.ConstantsForPID;
import org.firstinspires.ftc.teamcode.utility.DcMotorExW;
import org.firstinspires.ftc.teamcode.utility.MotorGroup;
import org.firstinspires.ftc.teamcode.utility.RunMotionProfile;

//TODO fix slides
public class PivotingSlide {

    private final MotorGroup slideMotors, pivotMotors;
    private double slideTarget = 0, pivotTarget = 0, slideOffset = 0;
    private final TouchSensor slideLimitSwitch;
    private final AnalogInput pivotEncoder;
    private final RunMotionProfile fastPivotProfile = new RunMotionProfile(30000,30000,30000,new ConstantsForPID(0.5,0,0.2,0,3,0));
    private final RunMotionProfile slowPivotProfile = new RunMotionProfile(10000,10000,10000,new ConstantsForPID(0.5,0,0.2,0,3,0));
    private final RunMotionProfile slideProfile = new RunMotionProfile(70000,70000,70000,new ConstantsForPID(0.2,0,0.2,0.2,2,0));
    private RunMotionProfile pivotProfile = fastPivotProfile;

    public final double MIN = -15, SET_POINT_1 = 300, SET_POINT_2 = 550, SET_POINT_3 = 700, MAX = 830;
    public enum States {
        MIN,
        SETPOINT_1,
        SETPOINT_2,
        SETPOINT_3,
        MAX
    }
    private States state = States.MIN;

    public PivotingSlide(HardwareMap hardwareMap, boolean slowPivot) {
        DcMotorExW liftLeft = new DcMotorExW(hardwareMap.get(DcMotorEx.class, "Llift"));
        DcMotorExW liftRight = new DcMotorExW(hardwareMap.get(DcMotorEx.class, "Rlift"));
        DcMotorExW pivotLeft = new DcMotorExW(hardwareMap.get(DcMotorEx.class, "Lpivot"));
        DcMotorExW pivotRight = new DcMotorExW(hardwareMap.get(DcMotorEx.class, "Rpivot"));
        liftRight.setPowerThresholds(0.05,0.05);
        liftLeft.setPowerThresholds(0.05,0.05);
        pivotRight.setPowerThresholds(0.05,0.05);
        pivotLeft.setPowerThresholds(0.05,0.05);

        pivotRight.setDirection(DcMotorSimple.Direction.REVERSE);

        slideLimitSwitch = hardwareMap.get(TouchSensor.class, "slideLimit");

        slideMotors = new MotorGroup(liftLeft, liftRight);
        pivotMotors = new MotorGroup(pivotLeft, pivotRight);

        resetEncoders();

        pivotEncoder = hardwareMap.get(AnalogInput.class, "pivotEncoder");
        if (slowPivot) pivotProfile = slowPivotProfile;
    }

    public void moveSlideTo(double target) {
        slideTarget = target;// - getCableDifference();
    }

    public void movePivotTo(double target) { pivotTarget = target; }

    public double getCableDifference() {
        return getPivotAngle() > 45 ? 30 : 0;
    }

    public void update() {
        if (slideLimitSwitch.isPressed()) {
            slideOffset = slideMotors.getPosition(0);
        }
        if (getSlidePosition() < 0) {
            resetEncoders();
        }
        slideMotors.setPowers(slideProfile.profiledMovement(slideTarget, getSlidePosition()));
        pivotMotors.setPowers(pivotProfile.profiledPivotMovement(pivotTarget, getPivotAngle()));
    }

    public void resetEncoders() {
        slideMotors.resetEncoders();
        slideOffset = 0;
    }

    public void disabledPIDsetPower(double power) {
        slideMotors.setPower(power,0);
        slideMotors.setPower(power,1);
    }

    public double getSlideError(){
        return slideTarget - getSlidePosition();
    }

    public double getSlidePosition() { return slideMotors.getPosition(0) - slideOffset; }

    public double getPivotAngle() { return  -pivotEncoder.getVoltage() * 72 + 203.5; }

    public boolean isTimeDone() { return slideProfile.getProfileDuration() + 0.5 < slideProfile.getCurrentTime(); }

    public boolean isPositionDone() { return Math.abs(getSlideError()) < 22; }

    public void setMotionConstraints(double maxVel, double maxAccel, double maxJerk){
        slideProfile.setMotionConstraints(maxVel, maxAccel, maxJerk);
    }

    public double getMotionTarget(){
        return slideProfile.getMotionTarget();
    }

    public double getSlideTarget() { return slideTarget; }

    public double getMotionTime() { return slideProfile.getCurrentTime(); }

    public void toMin() {
        moveSlideTo(MIN);
        state = States.MIN;
    }

    public void toSetPoint1() {
        moveSlideTo(SET_POINT_1);
        state = States.SETPOINT_1;
    }

    public void toSetPoint2() {
        moveSlideTo(SET_POINT_2);
        state = States.SETPOINT_2;
    }

    public void toSetPoint3() {
        moveSlideTo(SET_POINT_3);
        state = States.SETPOINT_3;
    }

    public void toMax() {
        moveSlideTo(MAX);
        state = States.MAX;
    }

    public States returnState() {
        return state;
    }

}
