package org.firstinspires.ftc.teamcode.teleop.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.Camera.Camera;
import org.firstinspires.ftc.teamcode.hardwaremaps.Robot;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain.AutoDrive;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain.GoToHeading;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.LiftArm.Lift;
import org.firstinspires.ftc.teamcode.subsystems.LiftArm.LiftHeight;

import java.util.List;
import java.util.concurrent.TimeUnit;


@Autonomous(name="Tuning Auto")
public class TuningAuto extends LinearOpMode {

    Robot robot;
    ElapsedTime elapsedTime = new ElapsedTime();
    Camera camera;
    LiftHeight duckHeight;

    @Override
    public void runOpMode() {

        robot = Robot.resetInstance();
        CommandScheduler.getInstance().reset();
        robot.init(hardwareMap, DriveTrain.DriveMode.AUTOIDLE,true);
        robot.lift.resetEncoder();
        robot.leftBack.resetEncoder();
        robot.liftArm.setHeight(LiftHeight.PICKUP);

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for(LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        while(!isStarted() && !isStopRequested()) {
            TelemetryPacket packet = new TelemetryPacket();

            packet.put("Forward Drive Target", robot.driveTrain.forwardPID.getTargetPosition());
            packet.put("Forward Drive Position", robot.leftFront.getEncoderCount());
            packet.put("Heading Target", robot.driveTrain.headingPID.getTargetPosition());
            packet.put("Current Heading",robot.driveTrain.currentHeading());
            packet.put("liftPID Lift Target", robot.liftArm.getPIDTarget());
            packet.put("LiftArm Lift Position", robot.lift.getEncoderCount());
            packet.put("liftPID Error", robot.liftArm.currentError);
            packet.put("0", 0);
            packet.put("5000", 0);
            packet.put("Elapsed Time(Milliseconds)", elapsedTime.time(TimeUnit.MILLISECONDS));
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }

        CommandScheduler.getInstance().schedule(new SequentialCommandGroup(
                //new AutoDrive(robot.driveTrain,1000)
                new GoToHeading(robot.driveTrain,-90),
                new WaitCommand(5000),
                //new Lift(robot.liftArm,LiftHeight.TOP),
                new GoToHeading(robot.driveTrain,0)
                //new AutoDrive(robot.driveTrain,1000)
        ));

        while(!isStopRequested()) {
            CommandScheduler.getInstance().run();
            TelemetryPacket packet = new TelemetryPacket();

            packet.put("Forward Drive Target", robot.driveTrain.forwardPID.getTargetPosition());
            packet.put("Forward Drive Position", robot.leftFront.getEncoderCount());
            packet.put("Heading Target", robot.driveTrain.headingPID.getTargetPosition());
            packet.put("Current Heading",robot.driveTrain.currentHeading());
            packet.put("liftPID Lift Target", robot.liftArm.getPIDTarget());
            packet.put("LiftArm Lift Position", robot.lift.getEncoderCount());
            packet.put("liftPID Error", robot.liftArm.currentError);
            packet.put("0", 0);
            packet.put("5000", 0);
            packet.put("Elapsed Time(Milliseconds)", elapsedTime.time(TimeUnit.MILLISECONDS));
            packet.put("Drive Train Operating Mode", robot.driveTrain.getDriveMode());
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }

        CommandScheduler.getInstance().cancelAll();
    }

}
