package org.firstinspires.ftc.teamcode.utility.wrappers;

import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/**
 * wrapper class for DcMotor which eliminates irrelevant hardware calls.
 * @see com.qualcomm.robotcore.hardware.DcMotorEx
 */
public class DcMotorExW implements DcMotorEx {
    private final DcMotorEx motor;
    private double lastPower = 0, powerStep = 0.05, minimum_power = 0.05;
    private boolean isZero = false;

    public DcMotorExW(DcMotorEx motor){
        this.motor = motor;
    }

    @Override
    public void setPower(double power) {
        if(!isZero && Math.abs(power) <= Math.abs(minimum_power)){
            isZero = true;
            lastPower = 0;
            motor.setPower(0);
        }
        else if(Math.abs(power - lastPower) >= powerStep && Math.abs(power) > Math.abs(minimum_power)){
            isZero = false;
            motor.setPower(power);
            lastPower = power;
        }
    }

    public void setPowerThresholds(double minimum_power, double powerStep){
        this.powerStep = powerStep;
        this.minimum_power = minimum_power;
    }

    @Override
    public void setMotorEnable() {
        motor.setMotorEnable();
    }

    @Override
    public void setMotorDisable() {
        motor.setMotorDisable();
    }

    @Override
    public boolean isMotorEnabled() {
        return motor.isMotorEnabled();
    }

    @Override
    public void setVelocity(double angularRate) {
        motor.setVelocity(angularRate);
    }

    @Override
    public void setVelocity(double angularRate, AngleUnit unit) {
        motor.setVelocity(angularRate, unit);
    }

    @Override
    public double getVelocity() {
        return motor.getVelocity();
    }

    @Override
    public double getVelocity(AngleUnit unit) {
        return motor.getVelocity(unit);
    }

    @Override
    @Deprecated
    public void setPIDCoefficients(RunMode mode, PIDCoefficients pidCoefficients) {
        motor.setPIDCoefficients(mode, pidCoefficients);
    }

    @Override
    public void setPIDFCoefficients(RunMode mode, PIDFCoefficients pidfCoefficients) throws UnsupportedOperationException {
        motor.setPIDFCoefficients(mode, pidfCoefficients);
    }

    @Override
    public void setVelocityPIDFCoefficients(double p, double i, double d, double f) {
        motor.setVelocityPIDFCoefficients(p, i, d, f);
    }

    @Override
    public void setPositionPIDFCoefficients(double p) {
        motor.setPositionPIDFCoefficients(p);
    }

    @Override
    @Deprecated
    public PIDCoefficients getPIDCoefficients(RunMode mode) {
        return motor.getPIDCoefficients(mode);
    }

    @Override
    public PIDFCoefficients getPIDFCoefficients(RunMode mode) {
        return motor.getPIDFCoefficients(mode);
    }

    @Override
    public void setTargetPositionTolerance(int tolerance) {
        motor.setTargetPositionTolerance(tolerance);
    }

    @Override
    public int getTargetPositionTolerance() {
        return motor.getTargetPositionTolerance();
    }

    @Override
    public double getCurrent(CurrentUnit unit) {
        return motor.getCurrent(unit);
    }

    @Override
    public double getCurrentAlert(CurrentUnit unit) {
        return motor.getCurrentAlert(unit);
    }

    @Override
    public void setCurrentAlert(double current, CurrentUnit unit) {
        motor.setCurrentAlert(current, unit);
    }

    @Override
    public boolean isOverCurrent() {
        return motor.isOverCurrent();
    }

    @Override
    public MotorConfigurationType getMotorType() {
        return motor.getMotorType();
    }

    @Override
    public void setMotorType(MotorConfigurationType motorType) {
        motor.setMotorType(motorType);
    }

    @Override
    public DcMotorController getController() {
        return motor.getController();
    }

    @Override
    public int getPortNumber() {
        return motor.getPortNumber();
    }

    @Override
    public void setZeroPowerBehavior(ZeroPowerBehavior zeroPowerBehavior) {
        motor.setZeroPowerBehavior(zeroPowerBehavior);
    }

    @Override
    public ZeroPowerBehavior getZeroPowerBehavior() {
        return motor.getZeroPowerBehavior();
    }

    @Override
    @Deprecated
    public void setPowerFloat() {
        motor.setPowerFloat();
    }

    @Override
    public boolean getPowerFloat() {
        return motor.getPowerFloat();
    }

    @Override
    public void setTargetPosition(int position) {
        motor.setTargetPosition(position);
    }

    @Override
    public int getTargetPosition() {
        return motor.getTargetPosition();
    }

    @Override
    public boolean isBusy() {
        return motor.isBusy();
    }

    @Override
    public int getCurrentPosition() {
        return motor.getCurrentPosition();
    }

    @Override
    public void setMode(RunMode mode) {
        motor.setMode(mode);
    }

    @Override
    public RunMode getMode() {
        return motor.getMode();
    }

    @Override
    public void setDirection(Direction direction) {
        motor.setDirection(direction);
    }

    @Override
    public Direction getDirection() {
        return motor.getDirection();
    }

    @Override
    public double getPower() {
        return motor.getPower();
    }

    @Override
    public Manufacturer getManufacturer() {
        return motor.getManufacturer();
    }

    @Override
    public String getDeviceName() {
        return motor.getDeviceName();
    }

    @Override
    public String getConnectionInfo() {
        return motor.getConnectionInfo();
    }

    @Override
    public int getVersion() {
        return motor.getVersion();
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {
        motor.resetDeviceConfigurationForOpMode();
    }

    @Override
    public void close() {
        motor.close();
    }

}
