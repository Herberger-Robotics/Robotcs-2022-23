package org.firstinspires.ftc.teamcode.subsystems.DriveTrain;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.hardwaremaps.Robot;

public class GoToHeading extends CommandBase {

    DriveTrain driveTrain;
    double adjustedAngle;
    double angle;

    public GoToHeading(DriveTrain driveTrain, double angle) {
        Robot robot = Robot.getInstance();
        this.driveTrain = driveTrain;
        this.angle  = angle;
    }

    @Override
    public void initialize() {
        Robot robot = Robot.getInstance();
        adjustedAngle = angle;
        driveTrain.setHeading(adjustedAngle);
        driveTrain.setDriveMode(DriveTrain.DriveMode.AUTOTURN);
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {
        //driveTrain.setDriveMode(DriveTrain.DriveMode.AUTOIDLE);
    }


    @Override
    public boolean isFinished() {
        Robot robot = Robot.getInstance();
        return Math.abs(-robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle - adjustedAngle) < 5;
    }



}
