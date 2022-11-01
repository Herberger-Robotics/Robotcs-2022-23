package org.firstinspires.ftc.teamcode.subsystems.LiftArm;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.hardwaremaps.Robot;

public class Lift extends CommandBase {

    private final LiftArm liftArm;
    private final LiftHeight height;

    public Lift(LiftArm liftArm, LiftHeight height) {
        this.liftArm = liftArm;
        this.height = height;
        addRequirements(liftArm);
    }

    @Override
    public void initialize() {
        liftArm.setHeight(height);
    }

    @Override
    public boolean isFinished() {
        Robot robot = Robot.getInstance();

        return Math.abs(liftArm.liftGet().getTargetPosition() - robot.lift.getEncoderCount()) < 100;
    }
}
