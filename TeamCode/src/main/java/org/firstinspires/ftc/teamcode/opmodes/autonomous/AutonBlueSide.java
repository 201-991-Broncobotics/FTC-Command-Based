package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.arcrobotics.ftclib.command.CommandOpMode;

import org.firstinspires.ftc.teamcode.Variables;
import org.firstinspires.ftc.teamcode.commands.utilcommands.DriveAndTurn;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.subsystems.Swerve;
import org.firstinspires.ftc.teamcode.subsystems.subsubsystems.DriveSubsystemBase;
@Autonomous(name = "23737 Auton (Blue Side)")
public class AutonBlueSide extends CommandOpMode {
    @Override
    public void initialize() {

        Variables.teleOp = false;
        boolean swerve_drive = true;
        Outtake outtake = new Outtake(hardwareMap);


        DriveSubsystemBase driveTrain;

        if (swerve_drive) {
            driveTrain = new Swerve(hardwareMap, telemetry, new String[] {
                    "rfm", "rbm", "lbm", "lfm"
            }, new String[] {
                    "rfs", "rbs", "lbs", "lfs"
            }, 12.913386, 9.133858, true,
                    RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                    RevHubOrientationOnRobot.UsbFacingDirection.DOWN
            );

            schedule(new SequentialCommandGroup(
                    new InstantCommand(() -> {
                        telemetry.addLine("Waiting for start");
                        telemetry.update();
                        waitForStart();
                        telemetry.addLine("Starting moving left and forward");
                        telemetry.update();
                    }),
                    new DriveAndTurn(driveTrain, 0, 20, -90),
                    new InstantCommand(() -> {
                        telemetry.addLine("Robot now moving right");
                        telemetry.update();
                    }),
                    new DriveAndTurn(driveTrain, 0, 5, 0),
                    new InstantCommand(() -> {
                        telemetry.addLine("Now moving left to and forward to the backdrop");
                        telemetry.update();
                    }),
                    new DriveAndTurn(driveTrain, 0, 5, -90),
                    new InstantCommand(() -> {
                        telemetry.addLine("Now inserting the pixels");
                        telemetry.update();
                    }),
                    new InstantCommand(outtake::shootOut),
                    new InstantCommand(() -> {
                        driveTrain.brake();
                        telemetry.addLine("Done with auto");
                        telemetry.update();
                    })
            ));

        // register subsystems

            register(driveTrain);

        // default commands


        }


    }
}
