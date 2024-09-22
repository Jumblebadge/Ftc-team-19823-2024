package org.firstinspires.ftc.teamcode.utility;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.maths.PID;
//TODO integrate PID
public class MotorGroup {

    public final DcMotorExW[] motors;

    public MotorGroup(DcMotorExW... motors){
        this.motors = motors;
    }

    public void setPowers(double... powers){
        for (int i = 0; i < motors.length; i++){
            motors[i].setPower(powers[i]);
        }
    }

    public void setPower(double power, int motor){
        motors[motor].setPower(power);
    }

    public double getPosition(int motor){
        return motors[motor].getCurrentPosition();
    }

    public double getAveragePosition(){
        double state = 0;

        for (DcMotorEx motor : motors) {
            state += motor.getCurrentPosition();
        }
        return state / motors.length;
    }


}
