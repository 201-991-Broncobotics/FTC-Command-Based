package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.PerpetualCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.ServoEx;

class SimpleServo extends SubsystemBase {
    private final ServoEx servo;
    private final Telemetry telem;
    private final String name;

    public SimpleServo(HardwareMap map, String name, Telemetry telem) {
        servo = new ServoEx(map, name, 0, 0, 300, 300, false);
        this.telem = telem;
        this.name = name;
    }

    public double getPower() {
        return servo.get();
    }

    public void powerForward() {
        servo.set(100);
    }

    public void powerBackward() {
        servo.set(-100);
    }

    public void stop() {
        servo.set(0);
    }

    @Override
    public void periodic() {
        servo.update();
        telem.addLine("\"" + name + "\" servo has current position " + servo.getRawPosition() + ". ");
    }
}

class SimpleMotor extends SubsystemBase {
    private final MotorEx motor;
    private final Telemetry telem;
    private final String name;

    public SimpleMotor(HardwareMap map, String name, Telemetry telem) {
        motor = new MotorEx(map, name, Motor.GoBILDA.RPM_312);
        motor.setRunMode(Motor.RunMode.RawPower);
        this.telem = telem;
        this.name = name;
    }

    public double getPower() {
        return motor.get();
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

    @Override
    public void periodic() {
        telem.addLine("\"" + name + "\" motor has current speed " + getPower() + ". ");
    }
}

@TeleOp(name = "Simple TeleOp Test")
public class SimpleTeleOpTest extends CommandOpMode {

    @Override
    public void initialize() {

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
