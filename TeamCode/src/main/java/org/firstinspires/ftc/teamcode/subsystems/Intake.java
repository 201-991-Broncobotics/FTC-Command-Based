package org.firstinspires.ftc.teamcode.subsystems;
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
        float leftIntake = gamepad2.left_trigger;
        float rightIntake = gamepad2.right_trigger;
        if (leftIntake > 0) {
            motor7.setPower(-1);
            motor8.setPower(-1);
        }
        if (rightIntake > 0) {
            motor7.setPower(1);
            motor8.setPower(1);
        }
        motor7.setPower(0);
        motor8.setPower(0);
    }
}

