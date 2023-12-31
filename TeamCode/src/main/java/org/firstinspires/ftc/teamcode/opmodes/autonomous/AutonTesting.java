package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.teamcode.Variables;
import org.firstinspires.ftc.teamcode.subsystems.Swerve;
import org.firstinspires.ftc.teamcode.subsystems.subsubsystems.DriveSubsystemBase;

@Autonomous(name = "Mael Tests Dumb Ideas")
public class AutonTesting extends CommandOpMode {
    @Override
    public void initialize() {

        Variables.teleOp = false;

        DriveSubsystemBase driveTrain = new Swerve(hardwareMap, telemetry,
                new String[] { // single swerve module lmao
                        "rfm", "rbm", "lbm", "lfm"
                }, new String[] {
                "rfs", "rbs", "lbs", "lfs"
        },14.173228 , 14.803150, true,
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.UP,
                "encoder"
        );
    }
    }

