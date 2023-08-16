package org.firstinspires.ftc.teamcode.opmodes.teleop;

import static org.firstinspires.ftc.teamcode.Constants.consecutive_trigger_time;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Test All Motors and Servos")
public class RunAllMotorsAndServos extends CommandOpMode {

    @Override
    public void initialize() {

        // initialize hardware

        MotorEx zero = new MotorEx(hardwareMap, "zero");
        MotorEx one = new MotorEx(hardwareMap, "one");
        MotorEx two = new MotorEx(hardwareMap, "two");
        MotorEx three = new MotorEx(hardwareMap, "three");
        MotorEx four = new MotorEx(hardwareMap, "four");
        MotorEx five = new MotorEx(hardwareMap, "five");
        MotorEx six = new MotorEx(hardwareMap, "six");
        MotorEx seven = new MotorEx(hardwareMap, "seven");

        // SimpleServo servo_one = new SimpleServo(hardwareMap, "servo_one", );

        GamepadEx driver = new GamepadEx(gamepad1), // start + a
                operator = new GamepadEx(gamepad2); // start + b

        Trigger forward = new Trigger(() -> driver.getButton(GamepadKeys.Button.DPAD_UP));
        Trigger reverse = new Trigger(() -> driver.getButton(GamepadKeys.Button.DPAD_DOWN));
        Trigger reset = new Trigger(() -> driver.getButton(GamepadKeys.Button.START));

        forward.whenActive(() -> {
            zero.set(0.5);
            one.set(0.5);
            two.set(0.5);
            three.set(0.5);
            one.set(0.5);
            four.set(0.5);
            five.set(0.5);
            six.set(0.5);
            seven.set(0.5);
        });

        reverse.whenActive(() -> {
            zero.set(-0.5);
            one.set(-0.5);
            two.set(-0.5);
            three.set(-0.5);
            one.set(-0.5);
            four.set(-0.5);
            five.set(-0.5);
            six.set(-0.5);
            seven.set(-0.5);
        });

        reset.whenActive(() -> {
            zero.set(0);
            one.set(0);
            two.set(0);
            three.set(0);
            one.set(0);
            four.set(0);
            five.set(0);
            six.set(0);
            seven.set(0);
        });
    }

}
