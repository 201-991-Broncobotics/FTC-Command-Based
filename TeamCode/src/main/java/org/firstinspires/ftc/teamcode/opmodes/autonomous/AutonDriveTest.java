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
import org.firstinspires.ftc.teamcode.commands.utilcommands.Wait;
import org.firstinspires.ftc.teamcode.subsystems.Mecanum;
import org.firstinspires.ftc.teamcode.subsystems.subsubsystems.DriveSubsystemBase;

@Autonomous(name = "Auton Drive Test")
public class AutonDriveTest extends CommandOpMode {

    @Override
    public void initialize() {

        // initialize variables

        Variables.teleOp = false;

        // Initialize Hardware

        DriveSubsystemBase driveTrain = new Mecanum(hardwareMap, telemetry, new String[] {
                "rf", "rb", "lb", "lf"
        }, new boolean[] {
                false, false, false, false
        }, 0, 0, 0, false,
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        ); // push straight forward to tune strafing

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
                new DriveAndTurn(driveTrain, 0, 20, 90),
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
                new DriveAndTurn(driveTrain, 0, 0, 0),
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
