package org.firstinspires.ftc.teamcode.subsystems.DriveTrain;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class TankDrive {

    MotorGroup leftMotorGroup;
    MotorGroup rightMotorGroup;

    public TankDrive(MotorGroup leftMotorGroup, MotorGroup rightMotorGroup) {
        this.leftMotorGroup = leftMotorGroup;
        this.rightMotorGroup = rightMotorGroup;
    }

    public static class MotorGroup {

        ArrayList<DcMotorEx> motors;

        public MotorGroup(DcMotorEx... motors) {
            this.motors = new ArrayList<>(Arrays.asList(motors));
        }

        public void set(double power) {
            motors.forEach(motor -> motor.setPower(power));
        }

        public void setDirection(DcMotorSimple.Direction direction) {
            motors.forEach(motor -> motor.setDirection(direction));
        }

        public void stop() {
            motors.forEach(motor -> motor.setPower(0));
        }

    }

}


