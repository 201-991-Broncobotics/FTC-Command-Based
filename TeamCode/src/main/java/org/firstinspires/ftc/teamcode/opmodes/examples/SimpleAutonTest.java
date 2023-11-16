package org.firstinspires.ftc.teamcode.opmodes.examples;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Variables;
import org.firstinspires.ftc.teamcode.commands.utilcommands.Wait;

@Disabled
@Autonomous(name = "Simple Auton Test")
public class SimpleAutonTest extends CommandOpMode {

    @Override
    public void initialize() {

        // initialize variables

        Variables.teleOp = false;

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
