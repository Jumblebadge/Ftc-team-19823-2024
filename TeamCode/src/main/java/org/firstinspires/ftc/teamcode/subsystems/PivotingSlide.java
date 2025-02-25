package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
    private final TouchSensor slideLimitSwitch;
    private final AnalogInput pivotEncoder;
    private RunMotionProfile pivotProfile = new RunMotionProfile(20000,10000,10000,new ConstantsForPID(0.6,0,1.5,0.5,1,0));
    private final RunMotionProfile slideProfile = new RunMotionProfile(125000,100000,100000,new ConstantsForPID(0.3,0,0.2,0.3,2,0));

    public final double MIN = -5, MAX = 675, SET_POINT_1 = MAX / 4, SET_POINT_2 = MAX / 2, SET_POINT_3 = 3 * MAX / 4;
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

        slideLimitSwitch = hardwareMap.get(TouchSensor.class, "slideLimit");

        slideMotors = new MotorGroup(lift1, lift2, lift3);

        resetEncoders();

        pivotEncoder = hardwareMap.get(AnalogInput.class, "pivotEncoder");
        if (slowPivot) {
            pivotProfile = new RunMotionProfile(3000, 3000, 3000, new ConstantsForPID(0.5, 0, 0.2, 0, 3, 0));
        }
    }

    public void moveSlideTo(double target) {
        slideTarget = target;
    }

    public void movePivotTo(double target) { pivotTarget = target; }

    public void update() {
        if (slideLimitSwitch.isPressed()) {
            slideOffset = slideMotors.getPosition(0);
        }
        if (getSlidePosition() < 0) {
            resetEncoders();
        }
        slideMotors.setPowers(slideProfile.profiledMovement(slideTarget, getSlidePosition()));
        pivot.setPower(pivotProfile.profiledPivotMovement(pivotTarget, getPivotAngle()));
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

    public double getMotionTarget(){
        return pivotProfile.getMotionTarget();
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
