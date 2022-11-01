package org.firstinspires.ftc.teamcode.subsystems.DuckWheel;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardwaremaps.Robot;

public class DuckWheel extends SubsystemBase {

    Robot robot;

    public DuckWheel(HardwareMap hwMap) {
        robot = Robot.getInstance();
        robot.duckMotor = new Motor(hwMap, "Duck Wheel", Motor.GoBILDA.RPM_1150);
        robot.duckMotor.setInverted(true);
        robot.duckMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
    }

    public void run() {
        robot.duckMotor.set(0.2);
    }

    public void stop() {
        robot.duckMotor.set(0);
    }

    public void runInverted() {
        robot.duckMotor.set(-0.2);
    }

}
