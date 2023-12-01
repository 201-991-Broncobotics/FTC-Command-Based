package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.Variables;
import org.firstinspires.ftc.teamcode.commands.utilcommands.SwerveDriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.Swerve;
import org.firstinspires.ftc.teamcode.subsystems.subsubsystems.CSensorBase;

@Autonomous(name = "23737 Auton - Test (Blue Side)")
public class BlueTest extends CommandOpMode {
    @Override
    public void initialize() {

        Variables.teleOp = false;

        //insert Subsystems
        CSensorBase CSensor = new CSensorBase(hardwareMap); //Puts in color sensor

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

            register(driveTrain);

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
            new SwerveDriveCommand(driveTrain,0,25,0),
            new InstantCommand(() -> {
                telemetry.addLine("Looking for Team Prop");
                telemetry.update();
                CSensor.GetTeamPropDistanceBLUE();
            }),
            new InstantCommand(() -> {
                if (CSensor.ColorFound) {
                    new InstantCommand(() -> {
                    telemetry.addLine("Team Prop Found!");
                    telemetry.addLine("Leaving Pixel Here");
                    telemetry.update();
                    });
                    new SequentialCommandGroup(() -> {
                        new SwerveDriveCommand(driveTrain, 0, -25, 0);
                        new SwerveDriveCommand(driveTrain, 0, 0, -90);
                        new SwerveDriveCommand(driveTrain, 0, 30, 0);
                        new SwerveDriveCommand(driveTrain, 20, 0, 0);
                        new SwerveDriveCommand(driveTrain, 0, 10, 0);
                        return null;
                    });
                    new InstantCommand(() -> {
                    driveTrain.brake();
                    CSensor.CSensorNotActive();
                    telemetry.addLine("Parked");
                    telemetry.addLine("Done with auto");
                    telemetry.update();
                    });
                } else {
                    new InstantCommand(() -> {
                    telemetry.addLine("Team Prop Not Found!");
                    telemetry.addLine("Looking elsewhere");
                    telemetry.update();
                    });
                    new SwerveDriveCommand(driveTrain,0,0,60);
                    new InstantCommand(CSensor::GetTeamPropDistanceBLUE2);
                    if (CSensor.ColorFound2) {
                        new InstantCommand(() -> {
                            telemetry.addLine("Team Prop Found!");
                            telemetry.addLine("Leaving Pixel Here");
                            telemetry.update();
                        });
                        new SequentialCommandGroup(() -> {
                            new SwerveDriveCommand(driveTrain,0,0,0);
                            new SwerveDriveCommand(driveTrain,0,-25,0);
                            new SwerveDriveCommand(driveTrain,0,0,-90);
                            new SwerveDriveCommand(driveTrain,0,30,0);
                            new SwerveDriveCommand(driveTrain,20,0,0);
                            new SwerveDriveCommand(driveTrain,0,10,0);
                            return null;
                        });
                        new InstantCommand(() -> {
                            driveTrain.brake();
                            CSensor.CSensorNotActive();
                            telemetry.addLine("Parked");
                            telemetry.addLine("Done with auto");
                            telemetry.update();
                        });
                    } else {
                        new InstantCommand(() -> {
                            telemetry.addLine("Team Prop Not Found!");
                            telemetry.addLine("Team Prop Must Be in Last Spot");
                            telemetry.update();
                        });
                        new SwerveDriveCommand(driveTrain,0,0,-90);
                        new InstantCommand(() -> telemetry.addLine("Leaving Pixel Here"));
                        new SequentialCommandGroup(() -> {
                            new SwerveDriveCommand(driveTrain,0,0,0);
                            new SwerveDriveCommand(driveTrain,0,-25,0);
                            new SwerveDriveCommand(driveTrain,0,0,-90);
                            new SwerveDriveCommand(driveTrain,0,30,0);
                            new SwerveDriveCommand(driveTrain,20,0,0);
                            new SwerveDriveCommand(driveTrain,0,10,0);
                            return null;
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
        }
    }