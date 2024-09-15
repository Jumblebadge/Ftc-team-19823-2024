package org.firstinspires.ftc.teamcode.maths;

import com.qualcomm.robotcore.util.Range;

public class GainScheduledPID {

    public final PID[] pids;

    public GainScheduledPID(PID... pids) { this.pids = pids; }

    public double pidOut(int PID, double error) {
        return pids[PID].pidOut(error);
    }

    public double ffOut(int PID, double error, double velocityTarget, double accelerationTarget) {
        return pids[PID].ffOut(error, velocityTarget, accelerationTarget);
    }

    public void setPIDgains(int PID, double Kp, double Kd, double Ki, double Kf, double Kl) {
        pids[PID].setPIDgains(Kp, Kd, Ki, Kf, Kl);
    }

    public void setFFgains(int PID, double Kv, double Ka, double Kstatic) {
        pids[PID].setFFgains(Kv, Ka, Kstatic);
    }

    public void toDefault() {
        for (PID pid : pids) {
            pid.toDefault();
        }
    }
}
