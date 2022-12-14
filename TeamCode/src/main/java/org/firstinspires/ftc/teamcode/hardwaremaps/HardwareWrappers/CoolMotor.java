package org.firstinspires.ftc.teamcode.hardwaremaps.HardwareWrappers;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class CoolMotor extends Motor {
    private DcMotorEx m_motor;
    private double resetVal;

    public static double TICKS_PER_REV;

    private String name;
    public String getName() { return name; }

    public CoolMotor(HardwareMap hMap, String name, double TPR) {
        m_motor = hMap.get(DcMotorEx.class, name);
        TICKS_PER_REV = TPR;
        this.name = name;
    }

    public void set(double speed) {
        m_motor.setPower(speed);
    }

    public double get() {
        return m_motor.getPower();
    }

    public void setInverted(boolean isInverted) {
        m_motor.setDirection(!isInverted ? DcMotor.Direction.FORWARD : DcMotor.Direction.REVERSE);
    }

    public boolean getInverted() {
        return m_motor.getDirection() == DcMotor.Direction.REVERSE;
    }

    public void disable() {
        m_motor.close();
    }

    public String getDeviceType() {
        return null;
    }

    public void pidWrite(double output) {
        set(output);
    }

    public void stopMotor() {
        set(0);
    }

    public void setDCZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        m_motor.setZeroPowerBehavior(behavior);
    }

    public double getEncoderCount() {
        return m_motor.getCurrentPosition();
    }

    public void resetEncoder() {
        m_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
//
    public double getNumRevolutions() {
        return getEncoderCount() / TICKS_PER_REV;
    }

    public void setTarget(int target){m_motor.setTargetPosition(target);}

    public boolean busy(){return m_motor.isBusy(); }

    public void runToPosition(){m_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);}

    public void runUsingEncoder(){m_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);}

    public void runWithoutEncoder(){m_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);}


    public double getVelocity() {
        return m_motor.getVelocity();
    }


    public void setVelocity(double velocity) {
        m_motor.setVelocity(velocity);
    }
}