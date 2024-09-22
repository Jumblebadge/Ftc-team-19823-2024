package org.firstinspires.ftc.teamcode.maths;

import android.util.Range;

public class ConstantsForPID {

    private double Kp, Kd, Ki, Kf, Kl, Kv, Ka, Ks;
    private final double KpS, KdS, KiS, KfS, KlS;
    private final double pointTunedAt;

    public ConstantsForPID(double Kp, double Kd, double Ki, double Kf, double Kl, double pointTunedAt) {
        this.Kp = Kp;
        this.Kd = Kd;
        this.Ki = Ki;
        this.Kf = Kf;
        this.Kl = Kl;

        KpS = Kp;
        KdS = Kd;
        KiS = Ki;
        KfS = Kf;
        KlS = Kl;

        this.pointTunedAt = pointTunedAt;
    }

    public void setPIDgains(double Kp, double Kd, double Ki, double Kf, double Kl) {
        this.Kp = Kp;
        this.Kd = Kd;
        this.Ki = Ki;
        this.Kf = Kf;
        this.Kl = Kl;
    }

    public void setFFgains(double Kv, double Ka, double Ks) {
        this.Kv = Kv;
        this.Ka = Ka;
        this.Ks = Ks;
    }

    public void toDefault() {
        this.Kp = KpS;
        this.Kd = KdS;
        this.Ki = KiS;
        this.Kf = KfS;
        this.Kl = KlS;
    }

    public double Kp() { return Kp; }
    public double Kd() { return Kd; }
    public double Ki() { return Ki; }
    public double Kf() { return Kf; }
    public double Kl() { return Kl; }

    public double Kv() { return Kv; }
    public double Ka() { return Ka; }
    public double Ks() { return Ks; }

    public double getPointTunedAt() { return pointTunedAt; }

}
