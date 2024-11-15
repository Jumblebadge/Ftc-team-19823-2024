package org.firstinspires.ftc.teamcode.maths;

import android.util.Range;

import com.arcrobotics.ftclib.util.InterpLUT;

import java.util.ArrayList;

public class GainScheduledPID {

    private final PID pid = new PID(0,0,0,0,0,0);
    private final InterpLUT Plut = new InterpLUT();
    private final InterpLUT Dlut = new InterpLUT();
    private final InterpLUT Ilut = new InterpLUT();
    private final InterpLUT Flut = new InterpLUT();
    private final InterpLUT Llut = new InterpLUT();

    public GainScheduledPID(ConstantsForPID... constants) {
        for (ConstantsForPID constant : constants) {
            addConstantsToLUT(constant);
        }
        Plut.createLUT();
        Dlut.createLUT();
        Ilut.createLUT();
        Flut.createLUT();
        Llut.createLUT();
    }

    public double pidOut(double reference, double state) {
        pid.setPIDgains(Plut.get(reference), Dlut.get(reference), Ilut.get(reference), Flut.get(reference), Llut.get(reference));
        return pid.pidOut(reference, state);
    }

    public double pidAngleOut(double reference, double state) {
        pid.setPIDgains(Plut.get(reference), Dlut.get(reference), Ilut.get(reference), Flut.get(reference), Llut.get(reference));
        return pid.pidAngleOut(reference, state);
    }

    public double pidPivotOut(double reference, double state) {
        pid.setPIDgains(Plut.get(reference), Dlut.get(reference), Ilut.get(reference), Flut.get(reference), Llut.get(reference));
        return pid.pidPivotOut(reference, state);
    }

    public double ffOut(double reference, double state, double velocityTarget, double accelerationTarget) {
        return pid.ffOut(reference, state, velocityTarget, accelerationTarget);
    }

    public double ffAngleOut(double reference, double state, double velocityTarget, double accelerationTarget) {
        return pid.ffAngleOut(reference, state, velocityTarget, accelerationTarget);
    }

    public void addConstantsToLUT(ConstantsForPID constants) {
        double pointTunedAt = constants.getPointTunedAt();
        Plut.add(pointTunedAt, constants.Kp());
        Dlut.add(pointTunedAt, constants.Kd());
        Ilut.add(pointTunedAt, constants.Ki());
        Flut.add(pointTunedAt, constants.Kf());
        Llut.add(pointTunedAt, constants.Kl());
    }
}
