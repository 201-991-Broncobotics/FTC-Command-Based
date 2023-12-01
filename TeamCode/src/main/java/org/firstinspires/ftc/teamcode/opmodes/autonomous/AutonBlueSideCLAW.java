package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.Variables;
import org.firstinspires.ftc.teamcode.commands.utilcommands.DriveAndTurn;
import org.firstinspires.ftc.teamcode.subsystems.Swerve;
import org.firstinspires.ftc.teamcode.subsystems.subsubsystems.CSensorBase;
import org.firstinspires.ftc.teamcode.subsystems.subsubsystems.DriveSubsystemBase;
import org.firstinspires.ftc.teamcode.subsystems.Claw;

@Autonomous(name = "23737 Auton - Old w/ Claw (Blue Side)")
public class AutonBlueSideCLAW extends CommandOpMode {
    @Override
    public void initialize() {

        Variables.teleOp = false;
        boolean swerve_drive = true;

        DriveSubsystemBase driveTrain;

        //insert Subsystems
        CSensorBase CSensor = new CSensorBase(hardwareMap); //Puts in color sensor
        Claw claw = new Claw(hardwareMap); //Puts in claw

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
                        telemetry.addLine("Waiting for start...");
                        telemetry.update();
                        waitForStart();
                        new InstantCommand(CSensor::CSensorStartUp);
                        telemetry.addLine("Turned on Color Sensor");
                        telemetry.addLine("Starting moving towards the team prop");
                        telemetry.update();
                    }),
                    new DriveAndTurn(driveTrain, 0, 25, 0),
                    new InstantCommand(() -> {
                        telemetry.addLine("Looking for Team Prop");
                        telemetry.update();
                    }),
                    new InstantCommand(() -> {
                        CSensor.GetTeamPropDistanceBLUE();
                        if (!CSensor.ColorFound) {
                            telemetry.addLine("Team Prop Not Found!");
                            telemetry.addLine("Looking elsewhere");
                            telemetry.update();
                            new DriveAndTurn(driveTrain, 0, 0, 60);
                            CSensor.GetTeamPropDistanceBLUE();
                            if (!CSensor.ColorFound) {
                                telemetry.addLine("Team Prop Not Found!");
                                telemetry.addLine("Looking elsewhere");
                                telemetry.update();
                                new InstantCommand(() -> new DriveAndTurn(driveTrain, 0, 0, -90));
                                CSensor.GetTeamPropDistanceBLUE();
                                telemetry.addLine("Team Prop Must Be Here");
                                telemetry.addLine("Leaving Pixel Here");
                                new InstantCommand(claw::Open);
                                telemetry.update();
                                new DriveAndTurn(driveTrain, 0, 0, 0);
                                sleep(100);
                                new DriveAndTurn(driveTrain, 0, -25, 0);
                                sleep(100);
                                new DriveAndTurn(driveTrain, 0, 0, -90);
                                sleep(100);
                                new DriveAndTurn(driveTrain, 0, 30, 0);
                                sleep(100);
                                new DriveAndTurn(driveTrain, 20, 0, 0);
                                sleep(100);
                                new DriveAndTurn(driveTrain, 0, 10, 0);
                                driveTrain.brake();
                                telemetry.addLine("Parked to backdrop");
                                telemetry.addLine("Done with auto");
                                telemetry.update();
                            } else if (CSensor.ColorFound) {
                                telemetry.addLine("Team Prop Found!");
                                telemetry.addLine("Leaving Pixel Here");
                                new InstantCommand(claw::Open);
                                telemetry.addLine("Moving towards backdrop and shutting off the Color Sensor");
                                telemetry.update();
                                new InstantCommand(CSensor::CSensorNotActive);
                                new DriveAndTurn(driveTrain, 0, 0, 0);
                                sleep(100);
                                new DriveAndTurn(driveTrain, 0, -25, 0);
                                sleep(100);
                                new DriveAndTurn(driveTrain, 0, 0, -90);
                                sleep(100);
                                new DriveAndTurn(driveTrain, 0, 30, 0);
                                sleep(100);
                                new DriveAndTurn(driveTrain, 20, 0, 0);
                                sleep(100);
                                new DriveAndTurn(driveTrain, 0, 10, 0);
                                driveTrain.brake();
                                telemetry.addLine("Parked");
                                telemetry.addLine("Done WIth Auto");
                                telemetry.update();
                            }
                        } else if (CSensor.ColorFound)
                            telemetry.addLine("Team Prop Found!");
                        telemetry.addLine("Leaving Pixel Here");
                        new InstantCommand(claw::Open);
                        telemetry.addLine("Moving towards backdrop and shutting off the Color Sensor");
                        telemetry.update();
                        new InstantCommand(CSensor::CSensorNotActive);
                        new DriveAndTurn(driveTrain, 0, -25, 0);
                        sleep(100);
                        new DriveAndTurn(driveTrain, 0, 0, -90);
                        sleep(100);
                        new DriveAndTurn(driveTrain, 0, 30, 0);
                                sleep(100);
                                new DriveAndTurn(driveTrain, 20, 0, 0);
                                sleep(100);
                                new DriveAndTurn(driveTrain, 0, 10, 0);
                                new InstantCommand(() -> {
                                    telemetry.addLine("Parked");
                                    new InstantCommand(claw::Close);
                                    telemetry.update();
                                });
                                new InstantCommand(() -> {
                                    driveTrain.brake();
                                    telemetry.addLine("Done with auto");
                                    telemetry.update();
                                });
                    })
            ));

        // register subsystems

            register(driveTrain);
        }
    }
}
