package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.PerpetualCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Variables;
import org.firstinspires.ftc.teamcode.commands.utilcommands.DriveAndTurn;
import org.firstinspires.ftc.teamcode.commands.utilcommands.SwerveDriveCommand;
import org.firstinspires.ftc.teamcode.commands.utilcommands.Wait;
import org.firstinspires.ftc.teamcode.subsystems.Swerve;

@Autonomous(name = "Auton 23737")
public class Auton23737 extends CommandOpMode {

    @Override
    public void initialize() {

        // initialize variables

        Variables.teleOp = false;

        // Initialize Hardware

        Swerve driveTrain = new Swerve(hardwareMap, telemetry,
                new String[] { // single swerve module lmao
                        "rfm", "rbm", "lbm", "lfm"
                }, new String[] {
                "rfs", "rbs", "lbs", "lfs"
        }, 12.913386, 9.133858, true,
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.UP,
                "encoder"
        );

        Trigger stop = new Trigger(() -> driveTrain.getOdometry()[1] > 5); // I don't recommend triggers in autonomous but you can do them

        // register subsystems

        register(driveTrain);

        // schedule commands

        schedule(new SequentialCommandGroup(
                new InstantCommand(() -> {
                    telemetry.addLine("Waiting for start");
                    telemetry.update();
                    waitForStart();
                    telemetry.addLine("Started moving forward and turning right");
                    telemetry.update();
                }),
                new SwerveDriveCommand(driveTrain, 10, 10, 90),
                new InstantCommand(() -> {
                    telemetry.addLine("Robot stopped");
                    telemetry.update();
                }),
                new ParallelDeadlineGroup(
                    new Wait(2),
                    new PerpetualCommand(new InstantCommand(() -> driveTrain.drive(0, 0, 0, false, 1)))
                ),
                new InstantCommand(() -> {
                    telemetry.addLine("Returning to origin");
                    telemetry.update();
                }),
                new DriveAndTurn(driveTrain, -10, -10, 0),
                new ParallelDeadlineGroup(
                    new Wait(0.25),
                    new PerpetualCommand(new InstantCommand(() -> driveTrain.drive(0, 0, 0, false, 1)))
                ),
                new InstantCommand(() -> {
                    driveTrain.brake();
                    telemetry.addLine("Done with auto");
                    telemetry.update();
                })
        ));
    }
}
