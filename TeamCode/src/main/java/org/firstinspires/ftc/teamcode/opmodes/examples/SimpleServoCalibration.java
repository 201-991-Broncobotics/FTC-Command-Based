package org.firstinspires.ftc.teamcode.opmodes.examples;

import static org.firstinspires.ftc.teamcode.Constants.normalize_joystick;
import static org.firstinspires.ftc.teamcode.Constants.round;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.PerpetualCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Variables;

import java.util.function.DoubleSupplier;

class SimpleServoDefaultCommand extends CommandBase {

    private final SimpleServo servo;
    private final DoubleSupplier rightY, slowdown;
    private final Telemetry telem;

    public SimpleServoDefaultCommand(SimpleServo servo, DoubleSupplier rightY, DoubleSupplier slowdown, Telemetry telem) {
        this.servo = servo;
        addRequirements(servo);

        this.slowdown = slowdown;
        this.rightY = rightY;
        this.telem = telem;
    }

    @Override
    public void initialize() { }

    @Override
    public void execute() {
        double target_speed = 300 * normalize_joystick(rightY.getAsDouble()) * (1 - 0.66 * slowdown.getAsDouble());
        servo.set(target_speed);
    }

    @Override
    public void end(boolean interrupted) { }

    @Override
    public boolean isFinished() {
        return false;
    }
}

@TeleOp(name = "Simple Servo Calibration")
public class SimpleServoCalibration extends CommandOpMode {

    @Override
    public void initialize() {

        // initialize variables

        Variables.teleOp = true;

        // initialize hardware

        SimpleServo servo = new SimpleServo(hardwareMap, "servo", telemetry);

        GamepadEx operator = new GamepadEx(gamepad1);

        // register subsystems

        register(servo);

        // default commands

        servo.setDefaultCommand(new SimpleServoDefaultCommand(
            servo,
            () -> operator.getLeftY(),
            () -> operator.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER),
            telemetry
        ));

        // non default commands

        schedule(new RunCommand(telemetry::update));
    }
}
