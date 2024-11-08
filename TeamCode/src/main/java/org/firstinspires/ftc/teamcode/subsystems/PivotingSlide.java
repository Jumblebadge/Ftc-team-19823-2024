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
//TODO
public class PivotingSlide {

    private final MotorGroup slideMotors, pivotMotors;
    private double slideTarget, pivotTarget, slideOffset, pivotOffset;
    private final TouchSensor slideLimitSwitch, pivotLimitSwitch;
    private final AnalogInput pivotEncoder;
    private final RunMotionProfile pivotProfile = new RunMotionProfile(1,1,1,new ConstantsForPID(0,0,0,0,1,0));
    private final RunMotionProfile slideProfile = new RunMotionProfile(60000,70000,80000,new ConstantsForPID(0,0,0,0,1,0));

    public final double MIN = 0, SET_POINT_1 = 1, SET_POINT_2 = 2, SET_POINT_3 = 3, MAX = 4;
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
        DcMotorExW pivotLeft = new DcMotorExW(hardwareMap.get(DcMotorEx.class, "Llift"));
        DcMotorExW pivotRight = new DcMotorExW(hardwareMap.get(DcMotorEx.class, "Rlift"));
        liftRight.setPowerThresholds(0.05,0.05);
        liftLeft.setPowerThresholds(0.05,0.05);
        pivotRight.setPowerThresholds(0.05,0.05);
        pivotLeft.setPowerThresholds(0.05,0.05);

        liftLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        pivotLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        slideLimitSwitch = hardwareMap.get(TouchSensor.class, "slideLimit");
        pivotLimitSwitch = hardwareMap.get(TouchSensor.class, "pivotLimit");

        slideMotors = new MotorGroup(liftLeft, liftRight);
        pivotMotors = new MotorGroup(pivotLeft, pivotRight);

        pivotEncoder = hardwareMap.get(AnalogInput.class, "pivotEncoder");
    }

    public void moveSlideTo(double target){
        slideTarget = target;
    }

    public void movePivotTo(double target) { pivotTarget = target; }

    public void update() {
        if (slideLimitSwitch.isPressed()) {
            slideOffset = slideMotors.getPosition(0);
        }
        if (pivotLimitSwitch.isPressed()) {
            pivotOffset = pivotEncoder.getVoltage() * 74.16;
        }
        slideMotors.setPowers(slideProfile.profiledMovement(slideTarget, getSlidePosition()));
        pivotMotors.setPowers(pivotProfile.profiledMovement(pivotTarget, getPivotAngle()));
    }

    public void disabledPIDsetPower(double power) {
        slideMotors.setPower(power,0);
        slideMotors.setPower(power,1);
    }

    public double getSlideError(){
        return slideTarget - getSlidePosition();
    }

    public double getSlidePosition() { return slideMotors.getPosition(0) - slideOffset; }

    public double getPivotAngle() { return  pivotEncoder.getVoltage() * 74.16 - pivotOffset; }

    public boolean isTimeDone() { return slideProfile.getProfileDuration() < slideProfile.getCurrentTime(); }

    public boolean isPositionDone() { return Math.abs(getSlideError()) < 10; }

    public void setMotionConstraints(double maxVel, double maxAccel, double maxJerk){
        slideProfile.setMotionConstraints(maxVel, maxAccel, maxJerk);
    }

    public double getMotionTarget(){
        return -slideProfile.getMotionTarget();
    }

    public double getSlideTarget() { return -slideTarget; }

    public double getMotionTime() { return slideProfile.getCurrentTime(); }

    public void toMin() {
        moveSlideTo(MIN);
        state = PossibleStates.MIN;
    }

    public void toSetPoint1() {
        moveSlideTo(SET_POINT_1);
        state = PossibleStates.SETPOINT_1;
    }

    public void toSetPoint2() {
        moveSlideTo(SET_POINT_2);
        state = PossibleStates.SETPOINT_2;
    }

    public void toSetPoint3() {
        moveSlideTo(SET_POINT_3);
        state = PossibleStates.SETPOINT_3;
    }

    public void toMax() {
        moveSlideTo(MAX);
        state = PossibleStates.MAX;
    }

    public PossibleStates returnState() {
        return state;
    }

}
