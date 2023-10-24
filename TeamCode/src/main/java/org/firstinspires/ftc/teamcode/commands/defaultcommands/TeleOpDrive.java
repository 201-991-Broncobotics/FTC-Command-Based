package org.firstinspires.ftc.teamcode.commands.defaultcommands;

import static org.firstinspires.ftc.teamcode.subsystems.subsubsystems.Functions.*;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.subsubsystems.DriveSubsystemBase;

import java.util.function.DoubleSupplier;

public class TeleOpDrive extends CommandBase {

    private final DriveSubsystemBase driveTrain;
    private final DoubleSupplier lsx, lsy, rsx, rsy, turning, damping, absolute_damping;

    private final boolean absolute_driving;

    public TeleOpDrive(DriveSubsystemBase driveTrain, GamepadEx driver, double translation_multiplier, double turning_multiplier, boolean absolute_driving) {
        this.driveTrain = driveTrain;
        addRequirements(driveTrain);

        this.lsx = () -> normalize_joystick(driver.getLeftX()) * translation_multiplier;
        this.lsy = () -> normalize_joystick(driver.getLeftY()) * translation_multiplier;
        this.rsx = () -> normalize_joystick(driver.getRightX()) * turning_multiplier;
        this.rsy = () -> normalize_joystick(-driver.getRightY()) * turning_multiplier;

        this.turning = () -> (normalize_joystick(driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)) - normalize_joystick(driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER))) * turning_multiplier;

        this.damping = () -> 1 + 2 * normalize_joystick(driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER));
        this.absolute_damping = () -> driver.getButton(GamepadKeys.Button.RIGHT_BUMPER) ? 3 : 1;

        this.absolute_driving = absolute_driving;
    }

    @Override
    public void initialize() { }

    @Override
    public void execute() {
        if (absolute_driving) {
            double rx = rsx.getAsDouble(), ry = rsy.getAsDouble();
            if (rx * rx + ry * ry > trigger_deadzone) {
                driveTrain.setTargetHeading(vectorToAngle(rx, ry));
            }
            driveTrain.drive(lsx.getAsDouble(), lsy.getAsDouble(), turning.getAsDouble(), absolute_damping.getAsDouble());
        } else {
            driveTrain.drive(lsx.getAsDouble(), lsy.getAsDouble(), rsx.getAsDouble(), damping.getAsDouble());
        }
    }

    @Override
    public void end(boolean interrupted) { }

    @Override
    public boolean isFinished() {
        return false;
    }
}