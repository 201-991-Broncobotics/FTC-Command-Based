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

@Autonomous(name = "Mael Tests Dumb Ideas")
public class AutonTesting extends CommandOpMode { //Red. I know it doesn't work rn but it SHOULD once Auton code gets fixed ;) (eventually)
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
                })));
                }
    }

