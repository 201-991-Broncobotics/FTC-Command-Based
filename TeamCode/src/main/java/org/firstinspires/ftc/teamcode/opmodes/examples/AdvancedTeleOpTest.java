package org.firstinspires.ftc.teamcode.opmodes.examples;

import static org.firstinspires.ftc.teamcode.subsystems.subsubsystems.Functions.*;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Variables;
import org.firstinspires.ftc.teamcode.commands.defaultcommands.TeleOpDrive;
import org.firstinspires.ftc.teamcode.commands.utilcommands.TriggerSequence;
import org.firstinspires.ftc.teamcode.commands.examplecommands.TurnForever;
import org.firstinspires.ftc.teamcode.commands.utilcommands.DriveAndTurn;
import org.firstinspires.ftc.teamcode.subsystems.Mecanum;

@Disabled
@TeleOp(name = "Advanced TeleOp Test")
public class AdvancedTeleOpTest extends CommandOpMode {

    @Override
    public void initialize() {

        // initialize variables

        Variables.teleOp = true;

        // initialize hardware

        Mecanum mecanum = new Mecanum(hardwareMap, telemetry, new String[] {
                "rf", "rb", "lb", "lf"
            }, new boolean[] {
                false, false, false, false
            }, 0, 0, 0, false,
            RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
            RevHubOrientationOnRobot.UsbFacingDirection.UP
        ); // push straight forward to tune strafing

        GamepadEx driver = new GamepadEx(gamepad1), // start + a
                  operator = new GamepadEx(gamepad2); // start + b

        // Initialize Trigger Sequences

        TriggerSequence konami_sequence = new TriggerSequence(
            consecutive_trigger_time,
            () -> operator.getLeftY() > 0.1 || operator.getRightY() < -0.1 /* || operator.getButton(GamepadKeys.Button.DPAD_UP)) */, // haven't figured out how to not make this conflict
            () -> !(operator.getLeftY() > 0.1 || operator.getRightY() < -0.1 || operator.getButton(GamepadKeys.Button.DPAD_UP)),
            () -> operator.getLeftY() > 0.1 || operator.getRightY() < -0.1 || operator.getButton(GamepadKeys.Button.DPAD_UP),
            () -> operator.getLeftY() < -0.1 || operator.getRightY() > 0.1 || operator.getButton(GamepadKeys.Button.DPAD_DOWN),
            () -> !(operator.getLeftY() < -0.1 || operator.getRightY() > 0.1 || operator.getButton(GamepadKeys.Button.DPAD_DOWN)),
            () -> operator.getLeftY() < -0.1 || operator.getRightY() > 0.1 || operator.getButton(GamepadKeys.Button.DPAD_DOWN),
            () -> operator.getLeftX() < -0.1 || operator.getRightX() < -0.1 || operator.getButton(GamepadKeys.Button.DPAD_LEFT),
            () -> operator.getLeftX() > 0.1 || operator.getRightX() > 0.1 || operator.getButton(GamepadKeys.Button.DPAD_RIGHT),
            () -> operator.getLeftX() < -0.1 || operator.getRightX() < -0.1 || operator.getButton(GamepadKeys.Button.DPAD_LEFT),
            () -> operator.getLeftX() > 0.1 || operator.getRightX() > 0.1 || operator.getButton(GamepadKeys.Button.DPAD_RIGHT),
            () -> operator.getButton(GamepadKeys.Button.B),
            () -> operator.getButton(GamepadKeys.Button.A),
            () -> operator.getButton(GamepadKeys.Button.START)
        );

        TriggerSequence double_a = new TriggerSequence(
            consecutive_trigger_time,
            () -> driver.getButton(GamepadKeys.Button.A) && konami_sequence.isInactive,
            () -> !(driver.getButton(GamepadKeys.Button.A)),
            () -> driver.getButton(GamepadKeys.Button.A)
        );

        Trigger switch_mode = new Trigger(() -> driver.getButton(GamepadKeys.Button.Y));

        Trigger reset_gyro = new Trigger(() -> driver.getButton(GamepadKeys.Button.START) && konami_sequence.isInactive);

        Trigger up = new Trigger(() -> driver.getButton(GamepadKeys.Button.DPAD_UP) && konami_sequence.isInactive);
            // can't be konami_sequence.slice(2) because it would also trigger for up on thing... hmm...
        Trigger down = new Trigger(() -> driver.getButton(GamepadKeys.Button.DPAD_DOWN) && konami_sequence.isInactive);
        Trigger left = new Trigger(() -> driver.getButton(GamepadKeys.Button.DPAD_LEFT) && konami_sequence.isInactive);
        Trigger right = new Trigger(() -> driver.getButton(GamepadKeys.Button.DPAD_RIGHT) && konami_sequence.isInactive);

        Trigger return_to_origin = new Trigger(() -> driver.getButton(GamepadKeys.Button.X));

        Trigger easter_egg = konami_sequence.getEndTrigger();

        Trigger slow_speed = double_a.slice(2); // not 1
        Trigger fast_speed = double_a.getEndTrigger();

        // register subsystems

        register(mecanum);

        // default commands

        mecanum.setDefaultCommand(new TeleOpDrive(
            mecanum,
            driver,
            1, 1, true
        ));

        // non default commands

        reset_gyro.whenActive(new InstantCommand(() -> mecanum.resetHeading(0)));

        switch_mode.whenActive(new InstantCommand(mecanum::toggleDriveMode));

        up.whenActive(new InstantCommand(() -> mecanum.setTargetHeading(0)));
        down.whenActive(new InstantCommand(() -> mecanum.setTargetHeading(180)));
        left.whenActive(new InstantCommand(() -> mecanum.setTargetHeading(-90)));
        right.whenActive(new InstantCommand(() -> mecanum.setTargetHeading(90)));

        return_to_origin.toggleWhenActive(new DriveAndTurn(mecanum, 0, 0, 0));

        easter_egg.toggleWhenActive(konami_sequence.triggerSequenceCommand(
            new TurnForever(mecanum)
        ));

        slow_speed.whenActive(double_a.triggerSequenceCommand(
            new InstantCommand(() -> org.firstinspires.ftc.teamcode.Variables.max_speed = 0.5
        )));

        fast_speed.whenActive(double_a.triggerSequenceCommand(
            new InstantCommand(() -> org.firstinspires.ftc.teamcode.Variables.max_speed = 1
        )));

        schedule(new RunCommand(() -> {
            telemetry.addData("konami active", !konami_sequence.isInactive);
            telemetry.addData("konami index", konami_sequence.trigger_number);

            telemetry.addData("fast active", !double_a.isInactive);
            telemetry.addData("fast index", double_a.trigger_number);

            telemetry.addData("max speed", org.firstinspires.ftc.teamcode.Variables.max_speed);
            telemetry.update();
        }));
    }
}
