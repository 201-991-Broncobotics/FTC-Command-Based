package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.Variables;
import org.firstinspires.ftc.teamcode.commands.defaultcommands.TeleOpDrive;
import org.firstinspires.ftc.teamcode.subsystems.Drone;
import org.firstinspires.ftc.teamcode.subsystems.Swerve;
import org.firstinspires.ftc.teamcode.subsystems.subsubsystems.DriveSubsystemBase;

@TeleOp(name = "TeleOp 23737 - Pre-Comp 2")
public class TeleOp23737PreComp2 extends CommandOpMode {

    @Override
    public void initialize() {

        // initialize variables

        Drone drone = new Drone(hardwareMap);
        Claw claw = new Claw(hardwareMap);
        Variables.teleOp = true;

        // initialize hardware



        DriveSubsystemBase driveTrain = new Swerve(hardwareMap, telemetry,
            new String[] { // single swerve module lmao
                "rfm", "rbm", "lbm", "lfm"
            }, new String[] {
                "rfs", "rbs", "lbs", "lfs"
            }, 12.913386, 9.133858, true,
            RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
            RevHubOrientationOnRobot.UsbFacingDirection.UP,
            "encoder"
        );

        Rev2mDistanceSensor dsensor = hardwareMap.get(Rev2mDistanceSensor.class, "distance_sensor");

        GamepadEx driver = new GamepadEx(gamepad1);
        GamepadEx operator = new GamepadEx(gamepad2);

        Trigger switch_mode = new Trigger(() -> driver.getButton(GamepadKeys.Button.Y));

        Trigger reset_gyro = new Trigger(() -> driver.getButton(GamepadKeys.Button.START));

        Trigger up = new Trigger(() -> driver.getButton(GamepadKeys.Button.DPAD_UP));
        Trigger down = new Trigger(() -> driver.getButton(GamepadKeys.Button.DPAD_DOWN));
        Trigger left = new Trigger(() -> driver.getButton(GamepadKeys.Button.DPAD_LEFT));
        Trigger right = new Trigger(() -> driver.getButton(GamepadKeys.Button.DPAD_RIGHT));
        Trigger droneMechanism = new Trigger(() -> operator.getButton(GamepadKeys.Button.DPAD_LEFT));
        Trigger closeClaw = new Trigger(() -> operator.getButton(GamepadKeys.Button.LEFT_BUMPER));
        Trigger openClaw = new Trigger(() -> operator.getButton(GamepadKeys.Button.LEFT_BUMPER));
        Trigger upClaw = new Trigger(() -> operator.getButton(GamepadKeys.Button.Y));
        Trigger downClaw = new Trigger(() -> operator.getButton(GamepadKeys.Button.A));

        // register subsystems. How - Mael

        register(driveTrain);
        register(drone);
        register(claw);

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
        droneMechanism.whenActive(new InstantCommand(drone::Endgame));
        droneMechanism.whenInactive(new InstantCommand(drone::notEndgame));
        closeClaw.whenActive(new InstantCommand(claw::Close));
        closeClaw.whenInactive(new InstantCommand(claw::InactiveClaw));
        openClaw.whenActive(new InstantCommand(claw::Open));
        openClaw.whenInactive(new InstantCommand(claw::InactiveClaw));
        upClaw.whenActive(new InstantCommand(claw::MoveUp));
        upClaw.whenInactive(new InstantCommand(claw::InactiveArm));
        downClaw.whenActive(new InstantCommand(claw::MoveDown));
        downClaw.whenInactive(new InstantCommand(claw::InactiveArm));

        schedule(new RunCommand(() -> telemetry.update()));


    }
}
