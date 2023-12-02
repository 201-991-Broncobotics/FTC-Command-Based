package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.Variables;
import org.firstinspires.ftc.teamcode.commands.utilcommands.SwerveDriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.Swerve;

@Autonomous(name = "Auton Simple w/ Claw - Blue")
public class SimpleClawBlue extends CommandOpMode {
    @Override
    public void initialize() {

        Claw claw = new Claw(hardwareMap);
        Arm arm = new Arm(hardwareMap);

        Variables.teleOp = false;

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

        waitForStart();

        schedule(new SequentialCommandGroup(
                        new SwerveDriveCommand(driveTrain,0,3,0),
                        new SwerveDriveCommand(driveTrain,0,0,-90),
                        new SwerveDriveCommand(driveTrain,0,25,0),
                        new SwerveDriveCommand(driveTrain,15,0,0),
                        new SwerveDriveCommand(driveTrain,0,10,0),
                        new InstantCommand(arm::AutonArm),
                        new InstantCommand(claw::Open)
        ));
    }
}

