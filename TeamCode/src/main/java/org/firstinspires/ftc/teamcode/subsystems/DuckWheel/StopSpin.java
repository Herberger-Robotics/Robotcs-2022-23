package org.firstinspires.ftc.teamcode.subsystems.DuckWheel;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.hardwaremaps.Robot;

public class StopSpin extends CommandBase {

    public StopSpin(DuckWheel duckWheel) {
        addRequirements(duckWheel);
    }

    @Override
    public void initialize() {
        Robot robot = Robot.getInstance();

        robot.duckMotor.set(0);
    }

}
