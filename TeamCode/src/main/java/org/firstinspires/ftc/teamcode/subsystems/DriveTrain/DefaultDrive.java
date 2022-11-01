package org.firstinspires.ftc.teamcode.subsystems.DriveTrain;

import com.arcrobotics.ftclib.command.CommandBase;

import java.util.function.DoubleSupplier;

public class DefaultDrive extends CommandBase {

    private final DriveTrain driveTrain;
    private final DoubleSupplier forward;
    private final DoubleSupplier turn;

    public DefaultDrive(DriveTrain driveTrain, DoubleSupplier forward, DoubleSupplier turn) {

        this.driveTrain = driveTrain;
        this.forward = forward;
        this.turn = turn;

    }

    @Override
    public void execute() {
        driveTrain.setManualDrive(forward.getAsDouble(), turn.getAsDouble());
    }

}
