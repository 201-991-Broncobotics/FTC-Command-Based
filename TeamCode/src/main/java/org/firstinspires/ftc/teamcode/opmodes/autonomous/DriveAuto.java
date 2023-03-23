package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.PerpetualCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.utilcommands.DriveAndTurn;
import org.firstinspires.ftc.teamcode.commands.utilcommands.Wait;
import org.firstinspires.ftc.teamcode.subsystems.Mecanum;

@Autonomous(name = "Drive Auto")
public class DriveAuto extends CommandOpMode {

    @Override
    public void initialize() {

        // Initialize Hardware

        Mecanum mecanum = new Mecanum(hardwareMap, telemetry, 0, 0, 0);

        Trigger stop = new Trigger(() -> mecanum.getOdometry()[1] > 5); // I don't recommend triggers in autonomous but you can do them

        // register subsystems

        register(mecanum);

        // schedule commands

        schedule(new SequentialCommandGroup(
                new InstantCommand(() -> {
                    telemetry.addLine("Waiting for start");
                    telemetry.update();
                    waitForStart();
                    telemetry.addLine("Started moving forward and turning right");
                    telemetry.update();
                }),
                new DriveAndTurn(mecanum, 0, 20, 90),
                new InstantCommand(() -> {
                    telemetry.addLine("Robot stopped");
                    telemetry.update();
                }),
                new ParallelDeadlineGroup(
                    new Wait(2),
                    new PerpetualCommand(new InstantCommand(() -> mecanum.drive(0, 0, 0, false)))
                ),
                new InstantCommand(() -> {
                    telemetry.addLine("Returning to origin");
                    telemetry.update();
                }),
                new DriveAndTurn(mecanum, 0, 0, 0),
                new ParallelDeadlineGroup(
                    new Wait(0.25),
                    new PerpetualCommand(new InstantCommand(() -> mecanum.drive(0, 0, 0, false)))
                ),
                new InstantCommand(() -> {
                    mecanum.brake();
                    telemetry.addLine("Done with auto");
                    telemetry.update();
                })
        ));
    }
}
