package org.firstinspires.ftc.teamcode.subsystems.LiftArm;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.TimeUnit;

import kotlin.jvm.Volatile;

public class CloseIntake extends CommandBase {

    private ElapsedTime timer = new ElapsedTime();
    LiftArm liftArm;

    public CloseIntake(LiftArm liftArm) {
        this.liftArm = liftArm;
    }

    @Override
    public void initialize() {
        liftArm.closeIntake();
        timer.reset();
    }
//
    @Override
    public boolean isFinished() {
        return timer.time(TimeUnit.MILLISECONDS) >= 500;
    }

}
