package org.firstinspires.ftc.teamcode.subsystems.DriveTrain;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.DifferentialDrive;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.hardwaremaps.Robot;
import org.firstinspires.ftc.teamcode.hardwaremaps.HardwareWrappers.CoolMotor;

public class DriveTrain extends SubsystemBase {

    @Config
    public static class DriveTrainConstants {
        public static PIDCoefficients FORWARD_PID = new PIDCoefficients(0.00075,0,0);
        public static double FORWARD_kStatic = 0.1;
        public static PIDCoefficients HEADING_PID = new PIDCoefficients(0.0025,0,0);
        public static double HEADING_kStatic = 0.0125;
    }

    private MotorGroup leftMotors;
    private MotorGroup rightMotors;
    public DifferentialDrive driveTrain;

    public DriveMode setDriveMode(DriveMode driveMode) {
        this.driveMode = driveMode;
        return this.driveMode;
    }

    public DriveMode getDriveMode() {
        return driveMode;
    }

    private double speed = 0.6;
    public void setSpeed(double speed) {
        this.speed = speed;
    }
    public double getSpeed() {
        return speed;
    }

    private DriveMode driveMode;

    private double manualForward = 0;
    private double manualTurn = 0;

    public PIDFController forwardPID;
    public PIDFController headingPID;
    double targetAngle = 0;
    double targetX = 0;

    Orientation angles;

    public enum DriveMode {
        MANUAL,
        AUTOIDLE,
        AUTOFORWARD,
        AUTOTURN,
    }

    public DriveTrain(final HardwareMap hardwareMap, DriveMode driveMode) {
        Robot robot = Robot.getInstance();

        forwardPID = new PIDFController(DriveTrainConstants.FORWARD_PID,0,0,DriveTrainConstants.FORWARD_kStatic);
        headingPID = new PIDFController(DriveTrainConstants.HEADING_PID,0,0,DriveTrainConstants.HEADING_kStatic);

        this.driveMode = driveMode;

        robot.rightBack = new CoolMotor(hardwareMap, "rightBack", 134.4);
        robot.rightFront = new CoolMotor(hardwareMap, "rightFront",134.4);
        robot.leftBack = new CoolMotor(hardwareMap, "leftBack",134.4);
        robot.leftFront = new CoolMotor(hardwareMap, "leftFront",134.4);

        robot.rightBack.setDCZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rightFront.setDCZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.leftBack.setDCZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.leftFront.setDCZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        robot.leftFront.setInverted(true);
        robot.leftBack.setInverted(true);
        robot.rightBack.setInverted(true);
        robot.rightFront.setInverted(true);



        leftMotors = new MotorGroup(robot.leftFront, robot.leftBack);
        rightMotors = new MotorGroup(robot.rightFront, robot.rightBack);

        driveTrain = new DifferentialDrive(leftMotors, rightMotors);
        driveTrain.setMaxSpeed(0.6);

        resetEncoders();
    }

    public void setManualDrive(double forward, double turn) {
        manualForward = forward;
        manualTurn = turn;
    }



    public void update() {
        Robot robot = Robot.getInstance();
        angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double forwardCalculation;
        double turnCalculation;

        switch (driveMode) {
            case MANUAL:
                forwardCalculation = manualForward;
                turnCalculation = manualTurn;
                break;
            case AUTOFORWARD:
                forwardCalculation = forwardPID.update(robot.leftFront.getEncoderCount());
                turnCalculation = 0;
                break;
            case AUTOTURN:
                forwardCalculation = 0;
                turnCalculation = headingPID.update(currentHeading());
                break;
            default:
                forwardCalculation = 0;
                turnCalculation = 0;
                break;
        }

        driveTrain.arcadeDrive(forwardCalculation, turnCalculation);
    }

    public void setHeading(double target) {
        targetAngle = target;
        headingPID.setTargetPosition(target);
    }

    public double currentHeading() {
        Robot robot = Robot.getInstance();

        double currentHeading;

        angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        //if(angles.firstAngle <= 0) currentHeading = 360 - Math.abs(angles.firstAngle);
        currentHeading = -angles.firstAngle;

        return currentHeading;
    }

    public void setX(double target) {
        this.targetX = target;
        forwardPID.setTargetPosition(target);
    }

    @Override
    public void periodic() {
        update();
    }

    public void resetEncoders() {
        Robot robot = Robot.getInstance();

        robot.leftBack.resetEncoder();
        robot.rightBack.resetEncoder();
        robot.leftFront.resetEncoder();
        robot.rightFront.resetEncoder();

    }

    public void slow() { driveTrain.setMaxSpeed(0.3); }
    public void fast() {
        driveTrain.setMaxSpeed(0.6);
    }


    public void stop()
    {
        Robot robot = Robot.getInstance();

        robot.rightBack.set(0);
        robot.leftBack.set(0);
        robot.rightFront.set(0);
        robot.leftFront.set(0);

    }



}

