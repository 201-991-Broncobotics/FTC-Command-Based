package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

class SimpleMotor extends SubsystemBase {
    private final MotorEx motor;

    public SimpleMotor(HardwareMap map, String name) {
        motor = new MotorEx(map, name, Motor.GoBILDA.RPM_312);
    }

    public void powerForward() {
        motor.set(0.5);
    }

    public void powerBackward() {
        motor.set(-0.5);
    }

    public void stop() {
        motor.set(0);
    }
}

@TeleOp(name = "Simple Motor Test")
public class SimpleMotorTest extends CommandOpMode {

    @Override
    public void initialize() {

        // initialize hardware

        SimpleMotor motor = new SimpleMotor(hardwareMap, "motor");

        GamepadEx operator = new GamepadEx(gamepad1);

        Trigger power_motor_positive = new Trigger(() -> operator.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.1);
        Trigger power_motor_negative = new Trigger(() -> operator.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.1);
        Trigger stop_motor = new Trigger(() -> operator.getButton(GamepadKeys.Button.Y));

        // register subsystems

        register(motor);

        // no default commands

        // non default commands

        power_motor_positive.toggleWhenActive(new InstantCommand(motor::powerForward));
        power_motor_negative.toggleWhenActive(new InstantCommand(motor::powerBackward));
        stop_motor.toggleWhenActive(new InstantCommand(motor::stop));

        schedule(new RunCommand(() -> { telemetry.update(); }));
    }

    /* I'm not sure if this is necessary
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        while (!isStopRequested() && opModeIsActive()) {
            idle();
        }
    } */
}
