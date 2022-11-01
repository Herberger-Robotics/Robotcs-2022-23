package org.firstinspires.ftc.teamcode.subsystems.LiftArm;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardwaremaps.Robot;
import org.firstinspires.ftc.teamcode.hardwaremaps.HardwareWrappers.CoolMotor;

public class LiftArm extends SubsystemBase {

    private PIDFController liftPID;
    public PIDFController liftGet() {
        return liftPID;
    }
    public PIDFController liftSet(PIDFController liftPID) {
        this.liftPID = liftPID;
        return this.liftPID;
    }
    public double getPIDTarget() {
        return liftPID.getTargetPosition();
    }

    private static LiftHeight liftHeight = LiftHeight.ZERO;
    public LiftHeight getHeight() { return liftHeight; }
    public void setHeight(LiftHeight liftHeight) { this.liftHeight = liftHeight; }
    public double currentError;
    public boolean PIDControl = true;
    public double calculation = 0;

    public LiftArm(final HardwareMap hwMap) {
        Robot robot = Robot.getInstance();


        robot.lift = new CoolMotor(hwMap, "lift", 134.4);
        robot.lift.setInverted(true);
        liftPID = new PIDFController(new PIDCoefficients(0.0015, 0, 0));

        robot.intake = new CRServo(hwMap,"box");

        currentError = 0;
    }

    private double speed = 0.6;

    public void setSpeed(double speed) {
        this.speed = speed;
    }

    public double getSpeed() {
        return speed;
    }

    public void liftController() {
        if(!PIDControl) return;
       Robot robot = Robot.getInstance();

        switch(liftHeight) {
            case ZERO:
                liftPID.setTargetPosition(0);
            case PICKUP:
                liftPID.setTargetPosition(100);
                break;
            case DRIVE:
                liftPID.setTargetPosition(1300);
                break;
            case BOTTOM:
                liftPID.setTargetPosition(3000);
                break;
            case MIDDLE:
                liftPID.setTargetPosition(6500);
                break;
            case TOP:
                liftPID.setTargetPosition(8500);
                break;
            case CAP:
                liftPID.setTargetPosition(10500);
                break;
        }
        calculation = liftPID.update(robot.lift.getEncoderCount());
        robot.lift.set(calculation);
        currentError = liftPID.getLastError();
    }

    @Override
    public void periodic() {
        liftController();
    }

    public void intake() {
        Robot.getInstance().intake.set(1);
    }

    public void intakeReversed() {
        Robot.getInstance().intake.set(-1);
    }

    public void stopIntake() { Robot.getInstance().intake.set(0);}

    public void resetEncoder() {
        Robot.getInstance().lift.resetEncoder();
    }



    public boolean isBusy() {
        Robot robot = Robot.getInstance();
        boolean isBusy;
        if (robot.lift.busy())
            isBusy = true;
        else isBusy = false;
        return isBusy;
    }


    public void stop()
    {
        Robot robot = Robot.getInstance();
        PIDControl = false;
        robot.lift.set(0);

    }
}

