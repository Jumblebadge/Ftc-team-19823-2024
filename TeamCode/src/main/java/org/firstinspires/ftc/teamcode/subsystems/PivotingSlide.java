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
//TODO switch to inches
//TODO fix slides
public class PivotingSlide {

    private final MotorGroup slideMotors, pivotMotors;
    private double slideTarget = 0, pivotTarget = 0, slideOffset = 0, pivotOffset = 0;
    private final TouchSensor slideLimitSwitch, pivotLimitSwitch;
    private final AnalogInput pivotEncoder;
    private final RunMotionProfile pivotProfile = new RunMotionProfile(3000,3000,3000,new ConstantsForPID(0.3,0,0.2,0.1,3,0));
    private final RunMotionProfile slideProfile = new RunMotionProfile(37500,42500,42500,new ConstantsForPID(0.4,0,0.1,0.3,1,0));

    public final double MIN = -10, SET_POINT_1 = 200, SET_POINT_2 = 450, SET_POINT_3 = 700, MAX = 915;
    public enum PossibleStates {
        MIN,
        SETPOINT_1,
        SETPOINT_2,
        SETPOINT_3,
        MAX
    }
    private PossibleStates state = PossibleStates.MIN;

    private final double DIAMETER = 1;
    private final double RATIO = 1;
    private final double CIRCUMFERENCE = DIAMETER * Math.PI;

    public PivotingSlide(HardwareMap hardwareMap) {
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
        pivotLimitSwitch = hardwareMap.get(TouchSensor.class, "pivotLimit");

        slideMotors = new MotorGroup(liftLeft, liftRight);
        pivotMotors = new MotorGroup(pivotLeft, pivotRight);

        pivotEncoder = hardwareMap.get(AnalogInput.class, "pivotEncoder");
    }

    public void moveSlideTo(double target) {
        slideTarget = target;// - getCableDifference();
    }

    public void movePivotTo(double target) { pivotTarget = target; }

    public double getCableDifference() {
        return getPivotAngle() / 90 * 30;
    }

    public void update() {
        if (slideLimitSwitch.isPressed()) {
            slideOffset = slideMotors.getPosition(0);
        }
        if (pivotLimitSwitch.isPressed()) {
            pivotOffset = -pivotEncoder.getVoltage() * 74.16 + 164;
        }
        //slideMotors.setPowers(slideProfile.profiledMovement(slideTarget, getSlidePosition()));
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

    public double getPivotAngle() { return  -pivotEncoder.getVoltage() * 74.16 - pivotOffset + 164; }

    public boolean isTimeDone() { return slideProfile.getProfileDuration() < slideProfile.getCurrentTime(); }

    public boolean isPositionDone() { return Math.abs(getSlideError()) < 10; }

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
