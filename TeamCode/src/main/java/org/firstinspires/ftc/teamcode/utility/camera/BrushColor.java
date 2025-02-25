package org.firstinspires.ftc.teamcode.utility.camera;

import com.qualcomm.robotcore.hardware.DigitalChannelImpl;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class BrushColor {

    private final DigitalChannelImpl pin0;
    private final DigitalChannelImpl pin1;

    private enum ColorDetection {
        BLUE,
        RED,
        YELLOW,
        NONE
    }

    public BrushColor(HardwareMap hardwareMap) {
        pin0 = hardwareMap.get(DigitalChannelImpl.class, "color0");
        pin1 = hardwareMap.get(DigitalChannelImpl.class, "color1");
    }

    public boolean getPin0State() {
        return pin0.getState();
    }

    public boolean getPin1State() {
        return pin1.getState();
    }

    public ColorDetection getDetection() {
        boolean pin0 = getPin0State();
        boolean pin1 = getPin1State();

        if (pin0 && pin1) return ColorDetection.YELLOW;
        else if (pin0) return ColorDetection.BLUE;
        else if (pin1) return ColorDetection.RED;
        else return ColorDetection.NONE;
    }

}
