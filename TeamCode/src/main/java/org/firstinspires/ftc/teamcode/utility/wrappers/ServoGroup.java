package org.firstinspires.ftc.teamcode.utility.wrappers;

public class ServoGroup {

    public final ServoImplExW[] servos;
    public ServoGroup(ServoImplExW... servos){
        this.servos = servos;
    }

    public void setPosition(double... positions){
        for (int i = 0; i < servos.length; i++){
            servos[i].setPosition(positions[i]);
        }
    }

    public void setPositions(double position, int servo){
        servos[servo].setPosition(position);
    }

    public double getPosition(int servo){
        return servos[servo].getPosition();
    }

    public void PWMrelease() {
        for (ServoImplExW servo : servos) {
            servo.setPwmDisable();
        }
    }


}
