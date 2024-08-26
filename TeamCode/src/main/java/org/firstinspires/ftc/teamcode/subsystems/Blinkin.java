package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Blinkin {

    private final RevBlinkinLedDriver blink;

    public Blinkin(HardwareMap hardwareMap){
        blink = hardwareMap.get(RevBlinkinLedDriver.class, "blink");
    }

    public void setPattern(BlinkinPattern pattern){
        blink.setPattern(pattern);
    }

}
