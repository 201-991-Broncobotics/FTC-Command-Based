package org.firstinspires.ftc.teamcode.opmodes.utility;

import static org.firstinspires.ftc.teamcode.subsystems.subsubsystems.Functions.*;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.PerpetualCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Variables;

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

        servo.setDefaultCommand(new PerpetualCommand(new InstantCommand(
            () -> servo.set(300 * normalize_joystick(operator.getLeftY()) * (1 - 0.66 * operator.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER))
        ))));

        // non default commands

        schedule(new RunCommand(telemetry::update));
    }
}
