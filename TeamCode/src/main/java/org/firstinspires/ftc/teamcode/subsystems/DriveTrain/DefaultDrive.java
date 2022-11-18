package org.firstinspires.ftc.teamcode.subsystems.DriveTrain;

import com.arcrobotics.ftclib.command.CommandBase;

import java.util.function.DoubleSupplier;

public class DefaultDrive extends CommandBase {

    private final DriveTrain driveTrain;
    private final DoubleSupplier forward;
    private final DoubleSupplier turn;
    private final DoubleSupplier strafe;

    public DefaultDrive(DriveTrain driveTrain, DoubleSupplier strafe, DoubleSupplier forward, DoubleSupplier turn) {

        this.driveTrain = driveTrain;
        this.forward = forward;
        this.turn = turn;
        this.strafe = strafe;
//
    }

    @Override
    public void execute() {
        driveTrain.setManualDrive(strafe.getAsDouble(), forward.getAsDouble(), turn.getAsDouble());
    }

}
