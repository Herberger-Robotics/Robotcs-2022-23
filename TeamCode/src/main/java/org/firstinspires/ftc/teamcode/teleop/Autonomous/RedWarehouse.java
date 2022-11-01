package org.firstinspires.ftc.teamcode.teleop.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Camera.TSEPosition;
import org.firstinspires.ftc.teamcode.Camera.Camera;
import org.firstinspires.ftc.teamcode.hardwaremaps.Robot;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain.AutoDrive;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain.GoToHeading;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.LiftArm.Lift;
import org.firstinspires.ftc.teamcode.subsystems.LiftArm.LiftHeight;

import java.util.List;
import java.util.concurrent.TimeUnit;


@Autonomous(name="Red Warehouse")
public class RedWarehouse extends LinearOpMode {

    Robot robot;
    ElapsedTime elapsedTime = new ElapsedTime();
    Camera camera;
    LiftHeight duckHeight;
    double duckDistance;

    @Override
    public void runOpMode() {

        robot = Robot.resetInstance();
        CommandScheduler.getInstance().reset();
        robot.init(hardwareMap, DriveTrain.DriveMode.AUTOIDLE,true);
        robot.lift.resetEncoder();
        robot.leftBack.resetEncoder();
        robot.liftArm.setHeight(LiftHeight.PICKUP);
        camera = new Camera(hardwareMap);
        TSEPosition duckPosition = camera.getPosition();

        while(!isStarted() && !isStopRequested()) {
            robot.clearBulkCache();
            TelemetryPacket packet = new TelemetryPacket();

            packet.put("X Error",robot.driveTrain);
            packet.put("DriveTrain Drive Position", robot.leftFront.getEncoderCount());
            packet.put("liftPID Lift Target", robot.liftArm.getPIDTarget());
            packet.put("LiftArm Lift Position", robot.lift.getEncoderCount());
            packet.put("liftPID Error", robot.liftArm.currentError);
            packet.put("Duck Position", camera.getPosition());
            packet.put("0", 0);
            packet.put("5000", 0);
            packet.put("Elapsed Time(Milliseconds)", elapsedTime.time(TimeUnit.MILLISECONDS));
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
            duckPosition = camera.getPosition();
        }

        elapsedTime.reset();
        camera.endCamera();

        //Detector
        switch(duckPosition) {
            case LEFT:
                duckHeight = LiftHeight.BOTTOM;
                duckDistance = 220;
                break;
            case CENTER:
                duckHeight = LiftHeight.MIDDLE;
                duckDistance = 240;
                break;
            case RIGHT:
                duckHeight = LiftHeight.TOP;
                duckDistance = 320;
                break;
        }

        CommandScheduler.getInstance().schedule(new SequentialCommandGroup(
                new Lift(robot.liftArm,LiftHeight.DRIVE),
                new AutoDrive(robot.driveTrain,750)
                        .withTimeout(5000)
                        .alongWith(new Lift(robot.liftArm,duckHeight)),
                new GoToHeading(robot.driveTrain,-45),
                new AutoDrive(robot.driveTrain,duckDistance)
                        .withTimeout(5000),
                new InstantCommand(robot.liftArm::intakeReversed),
                new WaitCommand(1000),
                new InstantCommand(robot.liftArm::stopIntake),
                new AutoDrive(robot.driveTrain,-duckDistance - 100)
                        .withTimeout(5000),
                new GoToHeading(robot.driveTrain,-90)
                        .withTimeout(5000),
                new AutoDrive(robot.driveTrain,-2000)
                        .withTimeout(5000)
                        .alongWith(new Lift(robot.liftArm,LiftHeight.DRIVE))
        ));

        telemetry.addData("Auto Status", "STARTED");
        telemetry.speak("AUTONOMOUS HAS STARTED");

        while(!isStopRequested()) {
            robot.clearBulkCache();
            TelemetryPacket packet = new TelemetryPacket();
            CommandScheduler.getInstance().run();

            packet.put("drivePID Error",robot.liftArm.currentError);
            packet.put("DriveTrain Drive Position", robot.leftFront.getEncoderCount());
            packet.put("liftPID Lift Target", robot.liftArm.getPIDTarget());
            packet.put("LiftArm Lift Position", robot.lift.getEncoderCount());
            packet.put("liftPID Error", robot.liftArm.currentError);
            packet.put("0", 0);
            packet.put("5000", 0);
            packet.put("Elapsed Time(Milliseconds)", elapsedTime.time(TimeUnit.MILLISECONDS));
            packet.put("Duck Position", camera.getPosition());
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }

        CommandScheduler.getInstance().cancelAll();
    }
}

