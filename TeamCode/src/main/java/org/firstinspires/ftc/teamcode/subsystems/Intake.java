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

    public final double SPIN_IN = 1, SPIN_OUT = -SPIN_IN;
    public final double LATCH_CLOSED = 0.31, LATCH_OPEN = 0.56;
    public final double WRIST_UP = 0.9, WRIST_MIDDLE = 0.48, WRIST_OPEN = 0.45, WRIST_DOWN = 0.425;
    public final double ROTATOR_0 = 0.78, ROTATOR_180 = 0.05;

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

        spin.setPwmRange(new PwmControl.PwmRange(500, 2500));

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

    public void setSpinPower(double power) { spin.setPower(power);
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
