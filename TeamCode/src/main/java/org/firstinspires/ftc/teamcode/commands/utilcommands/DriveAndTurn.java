package org.firstinspires.ftc.teamcode.commands.utilcommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Mecanum_Old;
import org.firstinspires.ftc.teamcode.subsystems.subsubsystems.DriveSubsystemBase;

public class DriveAndTurn extends CommandBase {

    private final DriveSubsystemBase driveTrain;
    private final double target_x, target_y, target_angle;

    private final double yaw_tolerance = 2, position_tolerance = 1.5; // degrees and inches, respectively
    private final double slowdown = 1.5; // go at 66% speed

    public DriveAndTurn(DriveSubsystemBase driveTrain, double target_x, double target_y, double target_angle) {
        this.driveTrain = driveTrain;
        addRequirements(driveTrain);

        this.target_x = target_x;
        this.target_y = target_y;
        this.target_angle = target_angle;
    }

    @Override
    public void initialize() {
        driveTrain.setTargetHeading(target_angle);
        driveTrain.setTargetPosition(target_x, target_y);
    }

    @Override
    public void execute() {
        driveTrain.drive(0, 0, 0, true, slowdown);
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            driveTrain.resetTargets();
        }
        driveTrain.brake();
    }

    @Override
    public boolean isFinished() {
        return (Math.abs(driveTrain.getHeadingError()) < yaw_tolerance) && (Math.abs(driveTrain.getPositionError()) < position_tolerance);
    }

}