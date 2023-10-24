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
import org.firstinspires.ftc.teamcode.subsystems.Mecanum;
import org.firstinspires.ftc.teamcode.subsystems.Swerve;
import org.firstinspires.ftc.teamcode.subsystems.subsubsystems.DriveSubsystemBase;

@TeleOp(name = "TeleOp Drive Test")
public class TeleOpDriveTest extends CommandOpMode {

    @Override
    public void initialize() {

        // initialize variables

        Variables.teleOp = true;

        // initialize hardware

        boolean swerve_drive = true; // make false for mecanum

        DriveSubsystemBase driveTrain;

        if (swerve_drive) {
            driveTrain = new Swerve(hardwareMap, telemetry, new String[]{ // single swerve module lmao
                    "rfm", "rbm", "lbm", "lfm"
            }, new String[]{
                    "rfs", "rbs", "lbs", "lfs"
            }, 12.913386, 9.133858, false,
                    RevHubOrientationOnRobot.LogoFacingDirection.UP,
                    RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
            );
        } else {
            driveTrain = new Mecanum(hardwareMap, telemetry, new String[]{
                    "rf", "rb", "lb", "lf"
            }, new boolean[]{
                    false, false, false, false
            }, 0, 0, 0, false,
                    RevHubOrientationOnRobot.LogoFacingDirection.UP, //FORWARD,
                    RevHubOrientationOnRobot.UsbFacingDirection.FORWARD //UP
            );
        }

        GamepadEx driver = new GamepadEx(gamepad1);

        Trigger switch_mode = new Trigger(() -> driver.getButton(GamepadKeys.Button.Y));

        Trigger reset_gyro = new Trigger(() -> driver.getButton(GamepadKeys.Button.START));

        Trigger up = new Trigger(() -> driver.getButton(GamepadKeys.Button.DPAD_UP));
        Trigger down = new Trigger(() -> driver.getButton(GamepadKeys.Button.DPAD_DOWN));
        Trigger left = new Trigger(() -> driver.getButton(GamepadKeys.Button.DPAD_LEFT));
        Trigger right = new Trigger(() -> driver.getButton(GamepadKeys.Button.DPAD_RIGHT));

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

        up.whenActive(new InstantCommand(() -> driveTrain.setTargetHeading(0)));
        down.whenActive(new InstantCommand(() -> driveTrain.setTargetHeading(180)));
        left.whenActive(new InstantCommand(() -> driveTrain.setTargetHeading(-90)));
        right.whenActive(new InstantCommand(() -> driveTrain.setTargetHeading(90)));

        schedule(new RunCommand(() -> {
            telemetry.update();
        }));
    }
}
