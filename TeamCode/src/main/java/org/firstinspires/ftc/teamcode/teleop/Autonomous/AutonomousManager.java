package org.firstinspires.ftc.teamcode.teleop.Autonomous;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.hardwaremaps.Robot;

import java.util.HashMap;
import java.util.Map;


/*
public class AutonomousManager extends SubsystemBase {

    public enum AutoLabels {
        BLUE_LEFT,
        BLUE_RIGHT,
        RED_LEFT,
        RED_RIGHT,
    }
    public AutoLabels currentAuto = AutoLabels.BLUE_LEFT;

    public Map<AutoLabels, SequentialCommandGroup> autos = new HashMap<>();

    public AutonomousManager() {
        Robot robot = Robot.getInstance();
        autos.put(AutoLabels.BLUE_LEFT, new BlueLeft(robot.driveTrain, robot.liftArm, robot.duckWheel));
        autos.put(AutoLabels.BLUE_RIGHT, new BlueRight(robot.driveTrain, robot.liftArm, robot.duckWheel));
        autos.put(AutoLabels.RED_LEFT, new RedLeft(robot.driveTrain, robot.liftArm, robot.duckWheel));
        autos.put(AutoLabels.RED_RIGHT, new RedRight(robot.driveTrain, robot.liftArm, robot.duckWheel));
    }

    public void cycleCommands() {
        AutoLabels autoToSet= AutoLabels.BLUE_LEFT;
        switch(currentAuto) {
            case BLUE_LEFT: autoToSet = AutoLabels.BLUE_RIGHT; break;
            case BLUE_RIGHT: autoToSet = AutoLabels.RED_LEFT; break;
            case RED_LEFT: autoToSet = AutoLabels.RED_RIGHT; break;
            case RED_RIGHT: autoToSet = AutoLabels.BLUE_LEFT; break;
        }
        currentAuto = autoToSet;
    }

    public SequentialCommandGroup getCurrentCommand() {
        return autos.get(currentAuto);
    }

}

*/
