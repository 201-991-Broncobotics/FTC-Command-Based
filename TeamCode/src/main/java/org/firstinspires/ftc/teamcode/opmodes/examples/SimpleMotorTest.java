package org.firstinspires.ftc.teamcode.opmodes.examples;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Variables;

@Disabled
@TeleOp(name = "Simple Motor Test")
public class SimpleMotorTest extends CommandOpMode {

    @Override
    public void initialize() {

        // initialize variables

        Variables.teleOp = true;

        // initialize hardware

        SimpleMotor motor = new SimpleMotor(hardwareMap, "motor", telemetry);

        GamepadEx operator = new GamepadEx(gamepad1);

        Trigger power_motor_positive = new Trigger(() -> operator.getLeftY() > 0.1 || operator.getRightY() < -0.1);
        Trigger power_motor_negative = new Trigger(() -> operator.getLeftY() < -0.1 || operator.getRightY() > 0.1);
        Trigger stop_motor = new Trigger(() -> operator.getButton(GamepadKeys.Button.START));

        // register subsystems

        register(motor);

        // default commands

        // non default commands

        power_motor_positive.whenActive(new ConditionalCommand(
            new InstantCommand(motor::powerForward), new InstantCommand(motor::stop), () -> motor.getPower() < 0.25
        ));
        power_motor_negative.whenActive(new ConditionalCommand(
            new InstantCommand(motor::powerBackward), new InstantCommand(motor::stop), () -> motor.getPower() > -0.25
        ));
        stop_motor.whenActive(new InstantCommand(motor::stop));

        schedule(new RunCommand(telemetry::update));
    }
}
