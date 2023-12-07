package org.firstinspires.ftc.teamcode.commands.utilcommands;

import static org.firstinspires.ftc.teamcode.subsystems.subsubsystems.Functions.*;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.Swerve;

public class SwerveDriveCommand extends SequentialCommandGroup {

    public SwerveDriveCommand(Swerve driveTrain, double delta_x, double delta_y, double target_angle) {
        addRequirements(driveTrain);

        addCommands(
            new angleModules(driveTrain, delta_x, delta_y),
            new driveDistance(driveTrain, delta_x, delta_y),
            new turnSwerve(driveTrain, target_angle)
        );
    }
}

class angleModules extends CommandBase {
    private final Swerve driveTrain;
    private final double target_angle;

    private final double module_tolerance = 5; // degrees

    private double target_module_angles;

    public angleModules(Swerve driveTrain, double delta_x, double delta_y) {
        this.driveTrain = driveTrain;
        addRequirements(driveTrain);

        this.target_angle = vectorToAngle(delta_x, delta_y);
    }

    @Override
    public void initialize() {
        driveTrain.resetTargetHeading();

        target_module_angles = target_angle - driveTrain.getHeading();
    }

    @Override
    public void execute() {
        driveTrain.setModuleAngles(target_module_angles);
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
        return driveTrain.getModuleError(target_module_angles) < module_tolerance; // each module is within 10 degrees of its goal
    }
}

class driveDistance extends CommandBase {
    private final Swerve driveTrain;
    private final double delta_x, delta_y, target_magnitude;

    private final double position_tolerance = 0.5; // inches
    private final double slowdown = 3; // go at 33% speed

    private double target_module_angles;

    public driveDistance(Swerve driveTrain, double delta_x, double delta_y) {
        this.driveTrain = driveTrain;
        addRequirements(driveTrain);

        this.target_magnitude = Math.sqrt(delta_x * delta_x + delta_y * delta_y) - position_tolerance;
        this.delta_x = delta_x * target_magnitude;
        this.delta_y = delta_y * target_magnitude;
    }

    @Override
    public void initialize() {
        driveTrain.resetEncoder();
        driveTrain.resetTargetHeading();
    }

    @Override
    public void execute() {
        driveTrain.drive(delta_x, delta_y, 0, true, slowdown);
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
        return Math.abs(driveTrain.readEncoderDistance()) > target_magnitude; // each module is within 10 degrees of its goal
    }
}

class turnSwerve extends CommandBase {
    private final Swerve driveTrain;
    private final double target_angle;

    private final double yaw_tolerance = 3; // degrees
    private final double slowdown = 1; // go at full speed

    public turnSwerve(Swerve driveTrain, double target_angle) {
        this.driveTrain = driveTrain;
        addRequirements(driveTrain);

        this.target_angle = target_angle;
    }

    @Override
    public void initialize() {
        driveTrain.setTargetHeading(target_angle);
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
        return Math.abs(driveTrain.getHeadingError()) < yaw_tolerance;
    }
}