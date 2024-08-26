package org.firstinspires.ftc.teamcode.utility;

import com.qualcomm.robotcore.hardware.Gamepad;

public interface ButtonMap {

    Gamepad gamepad = null;
    ButtonDetector x = new ButtonDetector(), o = new ButtonDetector(), s = new ButtonDetector(), t = new ButtonDetector();

    public boolean a();

}
