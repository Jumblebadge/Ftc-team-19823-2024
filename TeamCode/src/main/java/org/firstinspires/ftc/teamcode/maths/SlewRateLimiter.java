package org.firstinspires.ftc.teamcode.maths;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class SlewRateLimiter {

    private final ElapsedTime timer = new ElapsedTime();
    private double output = 0;

    public double rateLimit(double input, double r){
        double time = timer.seconds();
        output += Range.clip(input - output, -r * time, r * time);
        timer.reset();
        return output;
    }

}
