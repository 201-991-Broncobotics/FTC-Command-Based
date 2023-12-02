package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.Variables;
import org.firstinspires.ftc.teamcode.commands.utilcommands.DriveAndTurn;
import org.firstinspires.ftc.teamcode.subsystems.Swerve;
import org.firstinspires.ftc.teamcode.subsystems.subsubsystems.DriveSubsystemBase;
@Autonomous(name = "23737 Auton Old (Red Side)")
public class AutonRedSide extends CommandOpMode {
    @Override
    public void initialize() {

        Variables.teleOp = false;
        boolean swerve_drive = true;
        CRServo servo;
        servo = hardwareMap.get(CRServo.class, "box2");


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
                        telemetry.addLine("Starting moving right and forward");
                        telemetry.update();
                    }),
                    new DriveAndTurn(driveTrain, 0, 20, 90),
                    new InstantCommand(() -> {
                        telemetry.addLine("Robot now moving left");
                        telemetry.update();
                    }),
                    new DriveAndTurn(driveTrain, 0, 5, 0),
                    new InstantCommand(() -> {
                        telemetry.addLine("Now moving right to and forward to the backdrop");
                        telemetry.update();
                    }),
                    new DriveAndTurn(driveTrain, 0, 5, 90),
                    new InstantCommand(() -> {
                        telemetry.addLine("Now inserting the pixels");
                        telemetry.update();
                    }),
                    new InstantCommand(()-> {
                        servo.setPower(1);
                        try {
                            Thread.sleep(1000);
                        } catch (InterruptedException e) {
                            telemetry.addLine("Oops! Took too long!");
                            telemetry.update();
                        }
                        servo.setPower(-1);
                        try {
                            Thread.sleep(1000);
                        } catch (InterruptedException e) {
                            telemetry.addLine("Oops! Took too long!");
                            telemetry.update();
                        }
                        servo.setPower(0);
                    }),
                    new InstantCommand(() -> {
                        driveTrain.brake();
                        telemetry.addLine("Done with auto :3");
                        telemetry.update();
                    })
            ));

            // register subsystems

            register(driveTrain);

            // default commands


        }


    }
}
