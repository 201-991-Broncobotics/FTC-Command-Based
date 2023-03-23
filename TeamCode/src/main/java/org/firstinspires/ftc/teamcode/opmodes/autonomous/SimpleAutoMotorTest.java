package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.commands.utilcommands.Wait;

class SimpleAutoMotor extends SubsystemBase {
    private final MotorEx motor;

    public SimpleAutoMotor(HardwareMap map, String name) {
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

@Autonomous(name = "Simple Auto Motor Test")
public class SimpleAutoMotorTest extends CommandOpMode {

    @Override
    public void initialize() {

        // initialize hardware

        SimpleAutoMotor motor = new SimpleAutoMotor(hardwareMap, "motor");

        // register subsystems

        register(motor);

        // schedule commands

        schedule(new SequentialCommandGroup(
            new InstantCommand(() -> {
                telemetry.addLine("Waiting for start");
                telemetry.update();
                waitForStart();
                motor.powerForward();
                telemetry.addLine("Started powering motor forward");
                telemetry.update();
            }),
            new Wait(1),
            new InstantCommand(() -> {
                motor.stop();
                telemetry.addLine("Stopped motor");
                telemetry.update();
            }),
            new Wait(1),
            new InstantCommand(() -> {
                motor.powerBackward();
                telemetry.addLine("Started powering motor backward");
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
