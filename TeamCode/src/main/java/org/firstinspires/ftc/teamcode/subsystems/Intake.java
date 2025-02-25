package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.utility.wrappers.CRServoImplExW;
import org.firstinspires.ftc.teamcode.utility.wrappers.ServoImplExW;

public class Intake {

    private final ServoImplExW wrist, rotator, latch;
    private final CRServoImplExW spin;

    //private final AnalogInput color;

    public final double SPIN_IN = 0.5, SPIN_OUT = -0.5;
    public final double LATCH_CLOSED = 0.5, LATCH_OPEN = 0.5;
    public final double WRIST_UP = 0, WRIST_CLEAR = 0.8, WRIST_DOWN = 0.9;
    public final double ROTATOR_0 = 0.925, ROTATOR_45 = 0.65, ROTATOR_90 = 0.375;

    public Intake(HardwareMap hardwareMap) {
        spin = new CRServoImplExW(hardwareMap.get(CRServoImplEx.class, "spin"));

        latch = new ServoImplExW(hardwareMap.get(ServoImplEx.class, "latch"));
        wrist = new ServoImplExW(hardwareMap.get(ServoImplEx.class, "wrist"));
        rotator = new ServoImplExW(hardwareMap.get(ServoImplEx.class, "rotator"));

        spin.setThresholds(0.01, 0.05);

        latch.setPositionThreshold(0.002);
        wrist.setPositionThreshold(0.002);
        rotator.setPositionThreshold(0.002);

        //color = hardwareMap.get(AnalogInput.class, "color");
    }

    public double getRawColor() {
        //return color.getVoltage() / 3.3 * 360;
        return 0;
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
    public void setRotatorTo90() {
        setRotatorPosition(ROTATOR_90);
    }
    public void setRotatorTo45() {
        setRotatorPosition(ROTATOR_45);
    }
}
