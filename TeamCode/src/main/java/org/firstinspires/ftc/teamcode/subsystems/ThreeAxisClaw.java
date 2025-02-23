package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.utility.ServoImplExW;

public class ThreeAxisClaw {

    private final ServoImplExW claw, wrist, rotator;

    public final double CLAW_OPEN = 0.125, CLAW_CLOSE = 0.625;
    public final double WRIST_UP = 0, WRIST_CLEAR = 0.8, WRIST_DOWN = 0.9;
    public final double ROTATOR_0 = 0.925, ROTATOR_45 = 0.65, ROTATOR_90 = 0.375;

    public ThreeAxisClaw(HardwareMap hardwareMap) {
        claw = new ServoImplExW(hardwareMap.get(ServoImplEx.class, "claw"));
        wrist = new ServoImplExW(hardwareMap.get(ServoImplEx.class, "wrist"));
        rotator = new ServoImplExW(hardwareMap.get(ServoImplEx.class, "rotator"));

        claw.setPositionThreshold(0.002);
        wrist.setPositionThreshold(0.002);
        rotator.setPositionThreshold(0.002);
    }

    public void setClawPosition(double position) {
        claw.setPosition(position);
    }

    public void setWristPosition(double position) {
        wrist.setPosition(position);
    }

    public void setRotatorPosition(double position) {
        rotator.setPosition(position);
    }

    public void setClawOpen() {
        setClawPosition(CLAW_OPEN);
    }
    public void setClawClose() {
        setClawPosition(CLAW_CLOSE);
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
