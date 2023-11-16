package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.teamcode.Variables;
import org.firstinspires.ftc.teamcode.commands.defaultcommands.TeleOpDrive;
import org.firstinspires.ftc.teamcode.subsystems.Mecanum;
import org.firstinspires.ftc.teamcode.subsystems.Swerve;
import org.firstinspires.ftc.teamcode.subsystems.subsubsystems.DriveSubsystemBase;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp Drive Test, Mael Edition")
 public class TeleOpMaelVersion extends CommandOpMode {

    @Override
    public void initialize() {

        // initialize variables
        CRServo servo;
        servo = hardwareMap.get(CRServo.class, "box2");
        CRServo servo2;
        servo2 = hardwareMap.get(CRServo.class, "drone");
        DcMotor motor5;
        DcMotor motor6;
        DcMotor motor7;
        DcMotor motor8;
        motor5 = hardwareMap.get(DcMotor.class, "la");
        motor6 = hardwareMap.get(DcMotor.class, "ra");
        motor7 = hardwareMap.get(DcMotor.class, "lintake");
        motor8 = hardwareMap.get(DcMotor.class, "rintake");
        boolean leftIntake = gamepad2.left_bumper;
        boolean rightIntake = gamepad2.right_bumper;
        Variables.teleOp = true;

        // initialize hardware

        boolean swerve_drive = true; // make false for mecanum

        DriveSubsystemBase driveTrain;



        if (swerve_drive) {
            driveTrain = new Swerve(hardwareMap, telemetry, new String[] { // single swerve module lmao
                    "rfm", "rbm", "lbm", "lfm"
            }, new String[] {
                    "rfs", "rbs", "lbs", "lfs"
            }, 12.913386, 9.133858, true,
                    RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                    RevHubOrientationOnRobot.UsbFacingDirection.DOWN
            );
        } else {
            driveTrain = new Mecanum(hardwareMap, telemetry, new String[]{
                    "rf", "rb", "lb", "lf"
            }, new boolean[]{
                    false, false, false, false
            }, 0, 0, 0, false,
                    RevHubOrientationOnRobot.LogoFacingDirection.UP,
                    RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
            );
        }

        GamepadEx driver = new GamepadEx(gamepad1);

        Trigger switch_mode = new Trigger(() -> driver.getButton(GamepadKeys.Button.Y));

        Trigger reset_gyro = new Trigger(() -> driver.getButton(GamepadKeys.Button.START));

        Trigger up = new Trigger(() -> driver.getButton(GamepadKeys.Button.DPAD_UP));
        Trigger down = new Trigger(() -> driver.getButton(GamepadKeys.Button.DPAD_DOWN));
        Trigger left = new Trigger(() -> driver.getButton(GamepadKeys.Button.DPAD_LEFT));
        Trigger right = new Trigger(() -> driver.getButton(GamepadKeys.Button.DPAD_RIGHT));

        // register subsystems. How - Mael

        float leftInput = gamepad2.left_trigger;
        float rightInput = gamepad2.right_trigger;
        if (leftInput > 0) {
            motor5.setPower(-1);
            motor6.setPower(-1);
        }
        if (rightInput > 0) {
            motor5.setPower(1);
            motor6.setPower(1);
        }
        motor5.setPower(0);
        motor6.setPower(0);

        if (gamepad2.x) {

            servo.setPower(1);

            try {
                Thread.sleep(1000);
            } catch (InterruptedException e) {
                telemetry.addData("Oops!" , " Took Too Long!");
            }

            servo.setPower(-1);

            try {
                Thread.sleep(1000);
            } catch (InterruptedException e) {
                telemetry.addData("Oops!" , " Took Too Long");
            }

        }
        servo.setPower(0);
        if (gamepad2.b) {

            servo2.setPower(-.25);
            try {
                Thread.sleep(1000);
            } catch (InterruptedException e) {
                telemetry.addData("Oops " , "Took too long!");
            }
            servo2.setPower(.25);
            try {
                Thread.sleep(1000);
            } catch (InterruptedException e) {
                telemetry.addData("Oops " , "Took too long!");
            }
            servo2.setPower(0);

        }
        servo2.setPower(0);
        if (leftIntake) {
            motor7.setPower(-1);
            motor8.setPower(-1);
        }
        if (rightIntake) {
            motor7.setPower(1);
            motor8.setPower(1);
        }
        motor7.setPower(0);
        motor8.setPower(0);



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
