package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.utilcommands.Wait;
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
        // telem.addLine("\"" + name + "\" servo has current position " + servo.getRawPosition() + ". ");
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
        // telem.addLine("\"" + name + "\" motor has current speed " + getPower() + ". ");
    }
}

@Autonomous(name = "Simple Auto Test")
public class SimpleAutoTest extends CommandOpMode {

    @Override
    public void initialize() {

        // initialize hardware

        SimpleMotor motor = new SimpleMotor(hardwareMap, "motor", telemetry);
        SimpleServo servo = new SimpleServo(hardwareMap, "servo", telemetry);

        // register subsystems

        register(motor, servo);

        // schedule commands

        schedule(new SequentialCommandGroup(
            new InstantCommand(() -> {
                telemetry.addLine("Waiting for start");
                telemetry.update();
                waitForStart();
                motor.powerForward();
                servo.powerForward();
                telemetry.addLine("Started powering motor and servo forward");
                telemetry.update();
            }),
            new Wait(1),
            new InstantCommand(() -> {
                motor.stop();
                servo.stop();
                telemetry.addLine("Stopped motor and servo");
                telemetry.update();
            }),
            new Wait(1),
            new InstantCommand(() -> {
                motor.powerBackward();
                servo.powerBackward();
                telemetry.addLine("Started powering motor and servo backward");
                telemetry.update();
            }),
            new Wait(1),
            new InstantCommand(() -> {
                motor.stop();
                telemetry.addLine("Done with auto");
                telemetry.update();
            })
        ));
    }
}
