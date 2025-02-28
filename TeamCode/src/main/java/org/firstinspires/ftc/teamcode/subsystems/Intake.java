package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServoImplEx;
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
    public final double LATCH_CLOSED = 0.35, LATCH_OPEN = 0.575;
    public final double WRIST_UP = 0.93, WRIST_CLEAR = 0.8, WRIST_DOWN = 0.21;
    public final double ROTATOR_0 = 0.075, ROTATOR_90 = 0.45, ROTATOR_180 = 0.8;

    public Intake(HardwareMap hardwareMap) {
        spin = new CRServoImplExW(hardwareMap.get(CRServoImplEx.class, "spin"));

        latch = new ServoImplExW(hardwareMap.get(ServoImplEx.class, "latch"));
        wrist = new ServoImplExW(hardwareMap.get(ServoImplEx.class, "wrist"));
        rotator = new ServoImplExW(hardwareMap.get(ServoImplEx.class, "rotator"));

        spin.setThresholds(0.01, 0.05);

        latch.setPositionThreshold(0.002);
        wrist.setPositionThreshold(0.002);
        rotator.setPositionThreshold(0.002);

        wrist.setPwmRange(new PwmControl.PwmRange(500, 2500));

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

    public void setSpinPower(double power) { spin.setPower(power); }

    public void setLatchOpen() {
        setLatchPosition(LATCH_OPEN);
    }
    public void setLatchClose() {
        setLatchPosition(LATCH_CLOSED);
    }

    public void setWristUp() {
        setWristPosition(WRIST_UP);
    }
    public void setWristClear() {
        setWristPosition(WRIST_CLEAR);
    }
    public void setWristDown() {
        setWristPosition(WRIST_DOWN);
    }

    public void setRotatorTo0() {
        setRotatorPosition(ROTATOR_0);
    }
    public void setRotatorTo180() {
        setRotatorPosition(ROTATOR_180);
    }
    public void setRotatorTo90() {
        setRotatorPosition(ROTATOR_90);
    }

    public void setSpinIn() { setSpinPower(SPIN_IN); }
    public void setSpinOut() { setSpinPower(SPIN_OUT); }
    public void setSpin0() {setSpinPower(0); }
}
