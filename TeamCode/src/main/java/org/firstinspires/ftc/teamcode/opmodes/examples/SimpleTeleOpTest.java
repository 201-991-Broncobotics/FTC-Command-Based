package org.firstinspires.ftc.teamcode.opmodes.examples;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Variables;

@TeleOp(name = "Simple TeleOp Test")
public class SimpleTeleOpTest extends CommandOpMode {

    @Override
    public void initialize() {

        // initialize variables

        Variables.teleOp = true;

        // initialize hardware

        SimpleMotor motor = new SimpleMotor(hardwareMap, "motor", telemetry);
        SimpleServo servo = new SimpleServo(hardwareMap, "servo", telemetry);

        GamepadEx operator = new GamepadEx(gamepad1);

        Trigger power_motor_positive = new Trigger(() -> operator.getLeftY() > 0.1 || operator.getRightY() < -0.1);
        Trigger power_motor_negative = new Trigger(() -> operator.getLeftY() < -0.1 || operator.getRightY() > 0.1);
        Trigger stop_motor = new Trigger(() -> operator.getButton(GamepadKeys.Button.START));

        Trigger power_servo_positive = new Trigger(() -> operator.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.1);
        Trigger power_servo_negative = new Trigger(() -> operator.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.1);
        Trigger stop_servo = new Trigger(() -> operator.getButton(GamepadKeys.Button.START)); // should never be necessary

        // register subsystems

        register(motor, servo);

        // default commands

        // non default commands

        power_motor_positive.whenActive(new ConditionalCommand(
            new InstantCommand(motor::powerForward), new InstantCommand(motor::stop), () -> motor.getPower() < 0.25
        ));
        power_motor_negative.whenActive(new ConditionalCommand(
            new InstantCommand(motor::powerBackward), new InstantCommand(motor::stop), () -> motor.getPower() > -0.25
        ));
        stop_motor.whenActive(new InstantCommand(motor::stop));

        power_servo_positive.whenActive(new ConditionalCommand(
            new InstantCommand(servo::powerForward), new InstantCommand(servo::stop), () -> servo.getPower() < 25
        ));
        power_servo_negative.whenActive(new ConditionalCommand(
            new InstantCommand(servo::powerBackward), new InstantCommand(servo::stop), () -> servo.getPower() > -25
        ));
        stop_servo.whenActive(new InstantCommand(servo::stop));

        schedule(new RunCommand(telemetry::update));
    }
}
