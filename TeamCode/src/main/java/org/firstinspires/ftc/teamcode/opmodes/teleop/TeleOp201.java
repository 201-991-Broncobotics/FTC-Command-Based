package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Variables;
import org.firstinspires.ftc.teamcode.commands.defaultcommands.TeleOpDrive;
import org.firstinspires.ftc.teamcode.opmodes.examples.SimpleMotor;
import org.firstinspires.ftc.teamcode.subsystems.Mecanum;
import org.firstinspires.ftc.teamcode.subsystems.subsubsystems.DriveSubsystemBase;

@TeleOp(name = "TeleOp 201")
public class TeleOp201 extends CommandOpMode {

    @Override
    public void initialize() {

        // initialize variables

        Variables.teleOp = true;

        // initialize hardware

        DriveSubsystemBase driveTrain = new Mecanum(hardwareMap, telemetry, new String[] {
            "FrontRight", "BackRight", "BackLeft", "FrontLeft"
        }, new boolean[] {
            false, false, false, false
        }, 0, 0, 0, false,
            RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
            RevHubOrientationOnRobot.UsbFacingDirection.UP
        );

        SimpleMotor motor = new SimpleMotor(hardwareMap, "Arm", telemetry);

        GamepadEx driver = new GamepadEx(gamepad1);
        GamepadEx operator = new GamepadEx(gamepad2);

        Trigger switch_mode = new Trigger(() -> driver.getButton(GamepadKeys.Button.Y));
        Trigger reset_gyro = new Trigger(() -> driver.getButton(GamepadKeys.Button.START));

        Trigger power_motor_positive = new Trigger(() -> operator.getLeftY() > 0.1 || operator.getRightY() < -0.1);
        Trigger power_motor_negative = new Trigger(() -> operator.getLeftY() < -0.1 || operator.getRightY() > 0.1);
        Trigger stop_motor = new Trigger(() -> operator.getButton(GamepadKeys.Button.START));

        // register subsystems

        register(driveTrain);

        // default commands

        driveTrain.setDefaultCommand(new TeleOpDrive(
            driveTrain,
            driver,
            1, 1, false
        ));

        // non default commands

        reset_gyro.whenActive(new InstantCommand(() -> driveTrain.resetHeading(0)));

        switch_mode.whenActive(new InstantCommand(driveTrain::toggleDriveMode));

        schedule(new RunCommand(() -> {
            telemetry.update();
            motor.set(gamepad2.right_stick_y);
        }));
    }
}