package org.firstinspires.ftc.teamcode.subsystems.DuckWheel;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardwaremaps.Robot;

import java.util.concurrent.TimeUnit;

public class Spin extends CommandBase {

    DuckWheel duckWheel;
    ElapsedTime elapsedTime;
    double startTime;

    public enum Side {
        BLUE,
        RED
    }

    Side side;


    public Spin(DuckWheel duckWheel, ElapsedTime elapsedTime, Side side)  {
        this.duckWheel = duckWheel;
        this.elapsedTime = elapsedTime;
        this.side = side;
    }

    @Override
    public void initialize() {
        Robot robot = Robot.getInstance();

        startTime = elapsedTime.time(TimeUnit.SECONDS);
    }

    @Override
    public void execute() {
        Robot robot = Robot.getInstance();


        switch (side) {
            case BLUE: robot.duckMotor.set(-0.25); break;
            case RED: robot.duckMotor.set(0.25); break;
        }
    }

    @Override
    public void end(boolean interrupted) {
        Robot robot = Robot.getInstance();

        robot.duckMotor.set(0);
    }

}
