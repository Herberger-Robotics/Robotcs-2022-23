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
@Autonomous(name="Feet Breaker")
public class FeetBreaker extends LinearOpMode {

    Robot robot;
    ElapsedTime elapsedTime = new ElapsedTime();
    Camera camera;
    LiftHeight duckHeight;
    double duckDistance;

    @Override
    public void runOpMode() {
        robot = Robot.resetInstance();
        CommandScheduler.getInstance().reset();
        robot.init(hardwareMap, DriveTrain.DriveMode.AUTOIDLE, true);
        robot.lift.resetEncoder();
        //robot.ComandScheduler.reset
        robot.leftBack.resetEncoder();
        robot.leftFront.resetEncoder();
        robot.rightBack.resetEncoder();
        robot.rightFront.resetEncoder();
        robot.liftArm.setHeight(LiftHeight.plow);
        camera = new Camera(hardwareMap);
        TSEPosition DuckPosition = camera.getPosition();

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
        packet.put("Duck Position",duckPosition);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
        TSEPosition targetPosition = camera.getPosition();
    }

    elapsedTime.reset;
    camera.endCamera();
    switch(targetPosition) {
        case LEFT:
            duckHeight = LiftHeight.BOTTOM;
            duckDistance = 300;
            break;
        case CENTER:
            duckHeight = LiftHeight.MIDDLE;
            duckDistance = 280;
            break;
        case RIGHT:
            duckHeight = LiftHeight.TOP;
            duckDistance = 350;
            break;

    }
    CommandScheduler.getInstance().schedule(new SequentialCommandGroup();
            new Lift(robot.liftArm.LiftHeight.DRIVE);
            new AutoDrive(robot.driveTrain, 800);
                .withTimeout(5000)
                .alongWith(new Lift(robot.liftArm,duckHeight)),
            new GoToHeading(robot.driveTrain, 45);
            new AutoDrive(robot.driveTrain, targetDistance);
                 .withTimeout(5000)
            new InstantCommand(robot.liftArmcloseClaw)
            new WaitCommand(1000),

                new InstantCommand(robot.liftArm::stopIntake),

                new AutoDrive(robot.driveTrain,-duckDistance - 100)

                        .withTimeout(5000),

                new GoToHeading(robot.driveTrain,90)

                        .withTimeout(5000),

                new AutoDrive(robot.driveTrain,-2000)

                        .withTimeout(5000)
                        .alongWith(new Lift robot.liftarm.duckHeight);

}
    packet.put("0", 0);
            packet.put("5000", 0);
            packet.put("Elapsed Time(Milliseconds)", elapsedTime.time(TimeUnit.MILLISECONDS));
            packet.put("Drive Train Operating Mode", robot.driveTrain.getDriveMode());
            packet.put("Duck Position",duckPosition);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
            TSEPosition targetPosition = camera.getPosition();
            }


            }
