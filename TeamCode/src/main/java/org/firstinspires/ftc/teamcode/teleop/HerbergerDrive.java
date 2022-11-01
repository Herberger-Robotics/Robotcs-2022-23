 /* Copyright (c) 2017 FIRST. All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted (subject to the limitations in the disclaimer below) provided that
  * the following conditions are met:
  *
  * Redistributions of source code must retain the above copyright notice, this list
  * of conditions and the following disclaimer.
  *
  * Redistributions in binary form must reproduce the above copyright notice, this
  * list of conditions and the following disclaimer in the documentation and/or
  * other materials provided with the distribution.
  *
  * Neither the name of FIRST nor the names of its contributors may be used to endorse or
  * promote products derived from this software without specific prior written permission.
  *
  * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
  * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
  * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
  * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  */

 package org.firstinspires.ftc.teamcode.teleop;


 import com.acmerobotics.dashboard.FtcDashboard;
 import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
 import com.arcrobotics.ftclib.command.CommandScheduler;
 import com.arcrobotics.ftclib.command.InstantCommand;
 import com.arcrobotics.ftclib.command.SequentialCommandGroup;
 import com.arcrobotics.ftclib.gamepad.GamepadEx;
 import com.arcrobotics.ftclib.gamepad.GamepadKeys;
 import com.qualcomm.robotcore.eventloop.opmode.OpMode;
 import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
 import com.qualcomm.robotcore.util.ElapsedTime;


 import org.firstinspires.ftc.teamcode.hardwaremaps.Robot;
 import org.firstinspires.ftc.teamcode.subsystems.DriveTrain.DefaultDrive;
 import org.firstinspires.ftc.teamcode.subsystems.DriveTrain.DriveTrain;
 import org.firstinspires.ftc.teamcode.subsystems.DuckWheel.Spin;
 import org.firstinspires.ftc.teamcode.subsystems.LiftArm.Lift;
 import org.firstinspires.ftc.teamcode.subsystems.LiftArm.LiftHeight;

 @TeleOp(name="Herberger Drive", group="Iterative Opmode")
 public class HerbergerDrive extends OpMode
 {
     // Declare OpMode members.
     private ElapsedTime runtime = new ElapsedTime();
     Robot robot;

     GamepadEx driverOp = null;
     GamepadEx toolOp = null;

     @Override
     public void init() {
         robot = Robot.resetInstance();
         robot.init(hardwareMap, DriveTrain.DriveMode.MANUAL,true);

         //Gamepad Initialization
         driverOp = new GamepadEx(gamepad1);
         toolOp = new GamepadEx(gamepad2);

         // Tell the driver that initialization is complete.
         telemetry.addData("Status", "Initialized");

     }

     @Override
     public void init_loop() {
         robot.clearBulkCache();
         if(driverOp.getButton(GamepadKeys.Button.LEFT_BUMPER)) { robot.lift.resetEncoder(); telemetry.addData("LIFT","RESET"); }
     }

     @Override
     public void start() {
         runtime.reset();
         robot.liftArm.setHeight(LiftHeight.DRIVE);

         new DefaultDrive(robot.driveTrain, () -> driverOp.getLeftY(), () -> driverOp.getRightX()).schedule();
     }

     @Override
     public void loop() {
         robot.clearBulkCache();
         CommandScheduler scheduler = CommandScheduler.getInstance();
         TelemetryPacket packet = new TelemetryPacket();

         driverOp.getGamepadButton(GamepadKeys.Button.A)
                 .whenPressed(new InstantCommand(robot.liftArm::intake))
                 .whenReleased(new InstantCommand(robot.liftArm::stopIntake));
         driverOp.getGamepadButton(GamepadKeys.Button.X)
                 .whenPressed(new SequentialCommandGroup(
                         new InstantCommand(robot.liftArm::intake),
                         new Lift(robot.liftArm, LiftHeight.ZERO)
                 ))
                 .whenReleased(new SequentialCommandGroup(
                         new InstantCommand(robot.liftArm::stopIntake),
                         new Lift(robot.liftArm, LiftHeight.DRIVE)
                 ));
         driverOp.getGamepadButton(GamepadKeys.Button.B)
                 .whenPressed(new SequentialCommandGroup(
                         new InstantCommand(robot.liftArm::intake),
                         new Lift(robot.liftArm, LiftHeight.ZERO)
                 ))
                 .whenReleased(new SequentialCommandGroup(
                         new Lift(robot.liftArm, LiftHeight.DRIVE)
                 ));

         if(driverOp.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.2) robot.driveTrain.slow();
         else robot.driveTrain.fast();

         toolOp.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(
                 new Lift(robot.liftArm, LiftHeight.DRIVE));
         toolOp.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
                 new Lift(robot.liftArm, LiftHeight.BOTTOM));
         toolOp.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(
                 new Lift(robot.liftArm, LiftHeight.MIDDLE));
         toolOp.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
                 new Lift(robot.liftArm, LiftHeight.TOP));
         toolOp.getGamepadButton(GamepadKeys.Button.X)
                 .whenPressed(new Lift(robot.liftArm, LiftHeight.CAP));


         toolOp.getGamepadButton(GamepadKeys.Button.A)
                 .whenPressed(new InstantCommand(robot.liftArm::intakeReversed))
                 .whenReleased(new InstantCommand(robot.liftArm::stopIntake));
         toolOp.getGamepadButton(GamepadKeys.Button.B)
                 .whenPressed(new InstantCommand(robot.liftArm::intake))
                 .whenReleased(new InstantCommand(robot.liftArm::stopIntake));

         /*toolOp.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
              .whenHeld(new Spin(robot.duckWheel,runtime, Spin.Side.BLUE));
         toolOp.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                 //      .whenHeld(new Spin(robot.duckWheel,runtime, Spin.Side.RED));*/
         scheduler.run();
         packet.put("liftPID Error",robot.liftArm.currentError);
         packet.put("liftPID Target", robot.liftArm.getPIDTarget());
         packet.put("lift Position", robot.lift.getEncoderCount());
         packet.put("0", 0);
         packet.put("5000", 0);
         FtcDashboard.getInstance().sendTelemetryPacket(packet);
     }

     /*
      * Code to run ONCE after the driver hits STOP
      */
     @Override
     public void stop() {

         CommandScheduler.getInstance().cancelAll();
         robot.driveTrain.stop();
         robot.duckMotor.stopMotor();
         robot.lift.stopMotor();

     }


 }