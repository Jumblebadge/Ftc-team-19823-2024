/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode.utility.wrappers;

import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;

//copied from the sdk. wrapper to limit hardware calls
public class CRServoImplExW
{

    private double lastPower = 0;
    private double powerStep = 0.01;

    private double minPower = 0.05;

    private boolean isZero = false;

    private final CRServoImplEx servo;

    public CRServoImplExW(CRServoImplEx servo) {
        this.servo = servo;
    }

    public void setThresholds(double powerThreshold, double minPower) {
        this.powerStep = powerThreshold;
        this.minPower = minPower;
    }

    public void setPower(double power) {
        if(!isZero && Math.abs(power) <= Math.abs(minPower)){
            isZero = true;
            lastPower = 0;
            servo.setPower(0);
        }
        else if(Math.abs(power - lastPower) >= powerStep && Math.abs(power) > Math.abs(minPower)){
            isZero = false;
            servo.setPower(power);
            lastPower = power;
        }
    }

    public void setPwmDisable() {
        servo.setPwmDisable();
    }

    public void setDirection(DcMotorSimple.Direction direction) {
        servo.setDirection(direction);
    }

    public double getPower() {
        return servo.getPower();
    }

    public void setPwmRange(PwmControl.PwmRange range) {
        servo.setPwmRange(range);
    }

}
