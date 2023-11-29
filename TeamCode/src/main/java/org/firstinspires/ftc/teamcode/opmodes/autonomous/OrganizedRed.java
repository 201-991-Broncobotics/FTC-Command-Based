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

@Autonomous(name = "23737 Auton BACKUP (Red Side)")
public class OrganizedRed extends CommandOpMode {
    @Override
    public void initialize() {

        Variables.teleOp = false;
        boolean swerve_drive = true;

        DriveSubsystemBase driveTrain;

        //insert Subsystems
        CSensorBase CSensor = new CSensorBase(hardwareMap); //Puts in color sensor

        if (swerve_drive) {
            driveTrain = new Swerve(hardwareMap, telemetry, new String[]{
                    "rfm", "rbm", "lbm", "lfm"
            }, new String[]{
                    "rfs", "rbs", "lbs", "lfs"
            }, 12.913386, 9.133858, true,
                    RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                    RevHubOrientationOnRobot.UsbFacingDirection.DOWN
            );

            schedule(new SequentialCommandGroup(
            new InstantCommand(() -> {
                telemetry.addLine("Waiting for Start...");
                telemetry.update();
                waitForStart();
                CSensor.CSensorStartUp();
                telemetry.addLine("Color Sensor Initialized");
                telemetry.addLine("Now moving towards the Team Prop");
                telemetry.update();
            }),
            new InstantCommand(() -> {
                telemetry.addLine("Looking for Team Prop");
                telemetry.update();
                new DriveAndTurn(driveTrain,0,25,0);
                CSensor.GetTeamPropDistanceRED();
            }),
            new InstantCommand(() -> {
                if (CSensor.ColorFound = true) {
                    new InstantCommand(() -> {
                    telemetry.addLine("Team Prop Found!");
                    telemetry.addLine("Leaving Pixel Here");
                    telemetry.update();
                    });
                    new InstantCommand(() -> {
                    new DriveAndTurn(driveTrain,0,-25,0);
                    new DriveAndTurn(driveTrain,0,0,90);
                    new DriveAndTurn(driveTrain,0,30,0);
                    new DriveAndTurn(driveTrain,-20,0,0);
                    new DriveAndTurn(driveTrain,0,10,0);
                    });
                    new InstantCommand(() -> {
                    driveTrain.brake();
                    CSensor.CSensorNotActive();
                    telemetry.addLine("Parked");
                    telemetry.addLine("Done with auto");
                    telemetry.update();
                    });
                } else if (CSensor.ColorFound = false) {
                    new InstantCommand(() -> {
                    telemetry.addLine("Team Prop Not Found!");
                    telemetry.addLine("Looking elsewhere");
                    telemetry.update();
                    });
                    new InstantCommand(() -> {
                    new DriveAndTurn(driveTrain,0,0,60);
                    CSensor.GetTeamPropDistanceRED();
                    });
                    if (CSensor.ColorFound = true) {
                        new InstantCommand(() -> {
                            telemetry.addLine("Team Prop Found!");
                            telemetry.addLine("Leaving Pixel Here");
                            telemetry.update();
                        });
                        new InstantCommand(() -> {
                            new DriveAndTurn(driveTrain,0,0,0);
                            new DriveAndTurn(driveTrain,0,-25,0);
                            new DriveAndTurn(driveTrain,0,0,90);
                            new DriveAndTurn(driveTrain,0,30,0);
                            new DriveAndTurn(driveTrain,-20,0,0);
                            new DriveAndTurn(driveTrain,0,10,0);
                        });
                        new InstantCommand(() -> {
                            driveTrain.brake();
                            CSensor.CSensorNotActive();
                            telemetry.addLine("Parked");
                            telemetry.addLine("Done with auto");
                            telemetry.update();
                        });
                    } else if (CSensor.ColorFound = false) {
                        new InstantCommand(() -> {
                            telemetry.addLine("Team Prop Not Found!");
                            telemetry.addLine("Team Prop Must Be in Last Spot");
                            telemetry.update();
                        });
                        new InstantCommand(() -> {
                            new DriveAndTurn(driveTrain,0,0,-90);
                            CSensor.GetTeamPropDistanceRED();
                        });
                        new InstantCommand(() -> {
                            telemetry.addLine("Leaving Pixel Here");
                            new DriveAndTurn(driveTrain,0,0,0);
                            new DriveAndTurn(driveTrain,0,-25,0);
                            new DriveAndTurn(driveTrain,0,0,90);
                            new DriveAndTurn(driveTrain,0,30,0);
                            new DriveAndTurn(driveTrain,-20,0,0);
                            new DriveAndTurn(driveTrain,0,10,0);
                        });
                        new InstantCommand(() -> {
                            driveTrain.brake();
                            CSensor.CSensorNotActive();
                            telemetry.addLine("Parked");
                            telemetry.addLine("Done with auto");
                            telemetry.update();
                        });
                    }
                }
            })
            ));

            //Subsystems

            register(driveTrain);
        }
    }
}