package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.PerpetualCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.utilcommands.DriveAndTurn;
import org.firstinspires.ftc.teamcode.commands.utilcommands.SwerveDriveCommand;
import org.firstinspires.ftc.teamcode.Variables;
import org.firstinspires.ftc.teamcode.subsystems.Swerve;
import org.firstinspires.ftc.teamcode.subsystems.subsubsystems.DriveSubsystemBase;

@Autonomous(name = "Auton Simple - Blue")
public class SimpleAutonBlue extends CommandOpMode {
    @Override
    public void initialize() {

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
        schedule(new SequentialCommandGroup(
                new SwerveDriveCommand(driveTrain,0 , 2, 0),
                new SwerveDriveCommand(driveTrain,-30,0,-0)
        ));



    }
}
