package org.firstinspires.ftc.teamcode.commands.utilcommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.geometry.Translation2d;

import org.firstinspires.ftc.teamcode.subsystems.Mecanum;

import static org.firstinspires.ftc.teamcode.Constants.*;

public class DriveAndTurn extends CommandBase {

    private final Mecanum mecanum;
    private final double target_x, target_y, target_angle;

    public DriveAndTurn(Mecanum mecanum, double target_x, double target_y, double target_angle) {
        this.mecanum = mecanum;
        addRequirements(mecanum);

        this.target_x = target_x;
        this.target_y = target_y;
        this.target_angle = target_angle;
    }

    @Override
    public void initialize() {
        mecanum.setTargetHeading(target_angle);
        mecanum.setTargetPosition(target_x, target_y);
    }

    @Override
    public void execute() {
        mecanum.drive(0, 0, 0, true);
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            mecanum.resetOdometry();
        }
        mecanum.brake();
    }

    @Override
    public boolean isFinished() {
        return (Math.abs(mecanum.getError()) < yaw_tolerance) && (Math.abs(mecanum.getPositionError()) < position_tolerance);
    }

}