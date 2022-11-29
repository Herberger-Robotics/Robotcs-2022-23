package org.firstinspires.ftc.teamcode.subsystems.claw;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

public class Claw extends LinearOpMode {

    private CRServo claw;


    @Override
    public void runOpMode() {
        claw = hardwareMap.crservo.get("claw");
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.a) {
                claw.setPower(1);
            }
            if (gamepad1.b) {
                claw.setPower(-1);
            }
            if (gamepad1.atRest()) {
                claw.setPower(0);
            }
            telemetry.update();
        }
    }
}