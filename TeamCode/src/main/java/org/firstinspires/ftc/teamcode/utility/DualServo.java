package org.firstinspires.ftc.teamcode.utility;

import org.firstinspires.ftc.teamcode.maths.ConstantsForPID;

public class DualServo {

    private final ServoImplExW servo1, servo2;
    private final RunMotionProfile profile = new RunMotionProfile(0.1,0.1,0.1,new ConstantsForPID(0,0,0,0,1,0));
    private double lastTarget;

    public DualServo(ServoImplExW servo2, ServoImplExW servo1){
        this.servo1 = servo1;
        this.servo2 = servo2;
    }

    public void setPositionUsingProfile(double target){
        servo1.setPosition(profile.profiledServoMovement(target,servo1.getPosition()));
        servo2.setPosition(profile.profiledServoMovement(1-target,servo2.getPosition()));
    }

    public void setPosition(double target){
        if (lastTarget != target) {
            lastTarget = target;
            servo1.setPosition(1-target);
            servo2.setPosition(target);
        }
        else{ lastTarget = target;}
    }

    public void setMotionConstraints(double maxVel, double maxAccel, double maxJerk){
        profile.setMotionConstraints(maxVel, maxAccel, maxJerk);
    }

    public double getMotionTarget(){
        return profile.getMotionTarget();
    }

    public double getPosition(){
        return servo2.getPosition();
    }

    public void PWMrelease() {
        servo1.setPwmDisable();
        servo2.setPwmDisable();
    }

}
