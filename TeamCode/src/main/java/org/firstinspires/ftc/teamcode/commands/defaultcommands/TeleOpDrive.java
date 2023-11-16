package org.firstinspires.ftc.teamcode.commands.defaultcommands;

import static org.firstinspires.ftc.teamcode.subsystems.subsubsystems.Functions.*;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.subsystems.subsubsystems.DriveSubsystemBase;

import java.util.function.DoubleSupplier;

public class TeleOpDrive extends CommandBase {

    private final DriveSubsystemBase driveTrain;
    private final DoubleSupplier lsx, lsy, rsx, rsy, turning, damping, absolute_damping;

    private final boolean use_absolute_dampening;

    /**
     * @param use_absolute_dampening turn on the absolute dampening feature, which makes speed dampening a
     *                         toggle (on right bumper), instead of a slider (on right trigger)
     */
    public TeleOpDrive(DriveSubsystemBase driveTrain, GamepadEx driver, double translation_multiplier, double turning_multiplier, boolean use_absolute_dampening) {
        this.driveTrain = driveTrain;
        addRequirements(driveTrain);

        this.lsx = () -> normalize_joystick(driver.getLeftX()) * translation_multiplier;
        this.lsy = () -> normalize_joystick(driver.getLeftY()) * translation_multiplier;
        this.rsx = () -> normalize_joystick(driver.getRightX()) * turning_multiplier;
        this.rsy = () -> normalize_joystick(-driver.getRightY()) * turning_multiplier;

        this.turning = () -> (normalize_joystick(driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)) - normalize_joystick(driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER))) * turning_multiplier;

        // slows down based on how much RIGHT TRIGGER is pressed down
        // divide speed by damping (so if RIGHT TRIGGER is halfway pressed down, speed = 50%)
        this.damping = () -> 1 + 2 * normalize_joystick(driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER));

        // slows down by 33% when right bumper is held
        this.absolute_damping = () -> driver.getButton(GamepadKeys.Button.RIGHT_BUMPER) ? 3 : 1;

        this.use_absolute_dampening = use_absolute_dampening;
    }

    @Override
    public void initialize() { }

    @Override
    public void execute() {
        if (use_absolute_dampening) {
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