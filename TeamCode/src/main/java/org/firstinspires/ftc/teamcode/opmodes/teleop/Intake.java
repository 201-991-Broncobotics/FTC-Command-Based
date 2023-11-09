package org.firstinspires.ftc.teamcode.opmodes.teleop;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class Intake extends OpMode {
    DcMotor motor7;
    DcMotor motor8;

    @Override
    public void init() {
        motor7 = hardwareMap.get(DcMotor.class, "lintake");
        motor8 = hardwareMap.get(DcMotor.class, "rintake");
        telemetry.addData("Work", " Plz?");

    }

    @Override
    public void loop() {
        boolean leftIntake = gamepad1.left_bumper;
        boolean rightIntake = gamepad1.right_bumper;
        if (leftIntake) {
            motor7.setPower(-1);
            motor8.setPower(-1);
        }
        if (rightIntake) {
            motor7.setPower(1);
            motor8.setPower(1);
        }
        motor7.setPower(0);
        motor8.setPower(0);
    }
}

