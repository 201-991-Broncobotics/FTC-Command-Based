package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.utilcommands.SwerveDriveCommand;
import org.firstinspires.ftc.teamcode.Variables;
import org.firstinspires.ftc.teamcode.subsystems.Swerve;

@Autonomous(name = "DFF")
public class DFF extends CommandOpMode { //DFF = DriveForwardForever; I'm trying to get auton to work :skull:

    @Override
    public void initialize() {

        Variables.teleOp = false;

        Swerve driveTrain = new Swerve(hardwareMap, telemetry,
                new String[] {
                        "rfm", "rbm", "lbm", "lfm"
                }, new String[] {
                "rfs", "rbs", "lbs", "lfs" //There were never supposed to be 8 servos?
        }, 13.858268, 13.228346, true,
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.UP,
                "encoder"
        );

        register(driveTrain);

        while (!Variables.teleOp){
            new SwerveDriveCommand(driveTrain,0,80,0);
        }
    }
}
