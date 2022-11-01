package org.firstinspires.ftc.teamcode.subsystems.DriveTrain;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.hardwaremaps.Robot;

public class AutoDrive extends CommandBase {

    DriveTrain driveTrain;
    double distance;
    double adjustedDistance;
    boolean isSet = false;

    public AutoDrive(DriveTrain driveTrain, double distance) {
        Robot robot = Robot.getInstance();
        this.driveTrain = driveTrain;
        this.distance = distance;
    }

    @Override
    public void initialize() {
        Robot robot = Robot.getInstance();
        adjustedDistance = distance;
        driveTrain.setX(distance);
        driveTrain.setDriveMode(DriveTrain.DriveMode.AUTOFORWARD);
        robot.leftFront.resetEncoder();
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {
        driveTrain.setDriveMode(DriveTrain.DriveMode.AUTOIDLE);
    }

    @Override
    public boolean isFinished() {
        Robot robot = Robot.getInstance();
        return Math.abs(robot.leftFront.getEncoderCount() - adjustedDistance) < 20;
    }

}
