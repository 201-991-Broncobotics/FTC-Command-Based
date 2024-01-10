package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.teamcode.commands.utilcommands.SwerveDriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.Huskylens;
import org.firstinspires.ftc.teamcode.Variables;
import org.firstinspires.ftc.teamcode.subsystems.DSensor;
import org.firstinspires.ftc.teamcode.subsystems.Swerve;

@Autonomous(name = "Yuh huh (Blue)")
public class BlueMain extends CommandOpMode { //Red. I know it doesn't work rn but it SHOULD once Auton code gets fixed ;) (eventually)
    @Override
    public void initialize() {

        Variables.teleOp = false;
        DSensor dsensor = new DSensor(hardwareMap);
        Huskylens huskylens = new Huskylens(hardwareMap);

        Swerve driveTrain = new Swerve(hardwareMap, telemetry,
                new String[] { // single swerve module lmao
                        "rfm", "rbm", "lbm", "lfm"
                }, new String[] {
                "rfs", "rbs", "lbs", "lfs"
        },14.173228 , 14.803150, true,
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.UP,
                "encoder"
        );

        register(driveTrain);
        register(dsensor);
        register(huskylens);

        schedule(new SequentialCommandGroup(
                new InstantCommand(() -> {
                    telemetry.addLine("Waiting for Start...");
                    telemetry.update();
                    waitForStart();
                    huskylens.initCamera(true);
                    telemetry.addLine("Huskylens Initialized");
                    telemetry.addLine("Moving Towards the Center");
                    telemetry.update();
                }),
                new SwerveDriveCommand(driveTrain,0,10,0),
                new InstantCommand(() -> {
                    dsensor.getDsResultOne();
                    dsensor.getDsResultTwo();
                    dsensor.getDsResultThree();
                    dsensor.getDsResultFour();
                    dsensor.getMinDistance();
                    dsensor.getComparedDSOne();
                    dsensor.getComparedDSTwo();
                    dsensor.getComparedDSThree();
                })));
        if (dsensor.comparedDSOne == 0) {
            new InstantCommand(() -> {
                telemetry.addLine("Distance Sensor Found Team Prop to the Front");
                telemetry.update();
                telemetry.addLine("Huskylens Confirmed Team Prop is to the Front");
                telemetry.addLine("Leaving Team Prop Here...");
            });
            new SwerveDriveCommand(driveTrain,0,2,0);
            new SwerveDriveCommand(driveTrain,0,-10,0);
            new SwerveDriveCommand(driveTrain,0,0,-90);
            new SwerveDriveCommand(driveTrain,0,10,0);
            new InstantCommand(() -> {
                telemetry.addLine("Done with Auto");
                telemetry.update();
                dsensor.turnOff();
                driveTrain.brake();
            });
        } else if (dsensor.comparedDSTwo == 0) {
            telemetry.addLine("Distance Sensor Found Team Prop to the Left");
            new SwerveDriveCommand(driveTrain, 0, 0, -90);
            telemetry.addLine("Leaving Team Prop Here...");
            telemetry.update();
            new SwerveDriveCommand(driveTrain, 0, 1, 0);
            new SwerveDriveCommand(driveTrain, 0, -1, 0);
            new SwerveDriveCommand(driveTrain, 0, 0, 0);
            new SwerveDriveCommand(driveTrain, 0, -8, 0);
            new SwerveDriveCommand(driveTrain, 0, 0, -90);
            new SwerveDriveCommand(driveTrain, 0, 10, 0);
            new InstantCommand(() -> {
                telemetry.addLine("Done with Auto");
                telemetry.update();
                dsensor.turnOff();
                driveTrain.brake();
            });
        } else if (dsensor.comparedDSThree == 0) {
            telemetry.addLine("Distance Sensor Found Team Prop to the Right");
            new SwerveDriveCommand(driveTrain,0,0,90);
            telemetry.addLine("Leaving Team Prop Here...");
            telemetry.update();
            new SwerveDriveCommand(driveTrain, 0, 1, 0);
            new SwerveDriveCommand(driveTrain, 0, -1, 0);
            new SwerveDriveCommand(driveTrain, 0, 0, 0);
            new SwerveDriveCommand(driveTrain, 0, -8, 0);
            new SwerveDriveCommand(driveTrain, 0, 0, -90);
            new SwerveDriveCommand(driveTrain, 0, 10, 0);
            new InstantCommand(() -> {
                telemetry.addLine("Done with Auto");
                telemetry.update();
                dsensor.turnOff();
                driveTrain.brake();
            });
        }
    }}

