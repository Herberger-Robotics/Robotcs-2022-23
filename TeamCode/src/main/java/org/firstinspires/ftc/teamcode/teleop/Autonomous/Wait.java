package org.firstinspires.ftc.teamcode.teleop.Autonomous;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.robocol.Command;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.TimeUnit;

public class Wait extends CommandBase {

    ElapsedTime timer = new ElapsedTime();
    double time;

    Wait(double time) {
        this.time = time;
    }

    @Override
    public void initialize() {
        timer.reset();
    }

    @Override
    public boolean isFinished() {
        return timer.time(TimeUnit.MICROSECONDS) >= time;
    }

}
