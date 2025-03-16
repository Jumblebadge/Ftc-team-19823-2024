package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.utility.camera.BrushColor;
import org.firstinspires.ftc.teamcode.utility.wrappers.CRServoImplExW;
import org.firstinspires.ftc.teamcode.utility.wrappers.ServoImplExW;

public class Intake {

    private final ServoImplExW wrist, rotator, latch;
    private final CRServoImplExW spin;

    private final BrushColor color;

    public final double SPIN_IN = 0.75, SPIN_OUT = -SPIN_IN;
    public final double LATCH_CLOSED = 0.3, LATCH_OPEN = 0.57;
    public final double WRIST_UP = 1, WRIST_MIDDLE = 0.25, WRIST_OPEN = 0.2, WRIST_DOWN = 0.03;
    public final double ROTATOR_0 = 0.8, ROTATOR_180 = 0.05;

    public Intake(HardwareMap hardwareMap) {
        spin = new CRServoImplExW(hardwareMap.get(CRServoImplEx.class, "spin"));

        latch = new ServoImplExW(hardwareMap.get(ServoImplEx.class, "latch"));
        wrist = new ServoImplExW(hardwareMap.get(ServoImplEx.class, "wrist"));
        rotator = new ServoImplExW(hardwareMap.get(ServoImplEx.class, "rotator"));

        spin.setThresholds(0.01, 0.05);
        spin.setDirection(DcMotorSimple.Direction.REVERSE);

        latch.setPositionThreshold(0.002);
        wrist.setPositionThreshold(0.002);
        rotator.setPositionThreshold(0.002);

        //wrist.setPwmRange(new PwmControl.PwmRange(500, 2500));

        color = new BrushColor(hardwareMap);
    }

    public BrushColor.ColorDetection getColor() {
        return color.getDetection();
    }

    public void setLatchPosition(double position) {
        latch.setPosition(position);
    }

    public void setWristPosition(double position) {
        wrist.setPosition(position);
    }

    public void setRotatorPosition(double position) {
        rotator.setPosition(position);
    }

    public void setSpinPower(double power) { //spin.setPower(power);
    }

    public void setLatchOpen() {
        setLatchPosition(LATCH_OPEN);
    }
    public void setLatchClose() {
        setLatchPosition(LATCH_CLOSED);
    }

    public void setWristUp() {
        setWristPosition(WRIST_UP);
    }
    public void setWristMiddle() { setWristPosition(WRIST_MIDDLE); }
    public void setWristOpen() { setWristPosition(WRIST_OPEN); }
    public void setWristDown() {
        setWristPosition(WRIST_DOWN);
    }

    public void setRotatorTo0() {
        setRotatorPosition(ROTATOR_0);
    }
    public void setRotatorTo180() {
        setRotatorPosition(ROTATOR_180);
    }

    public void setSpinIn() { setSpinPower(SPIN_IN); }
    public void setSpinOut() { setSpinPower(SPIN_OUT); }
    public void setSpin0() { setSpinPower(0); }
}
