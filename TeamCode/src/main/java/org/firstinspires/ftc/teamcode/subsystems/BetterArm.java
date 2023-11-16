package org.firstinspires.ftc.teamcode.subsystems;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class BetterArm extends OpMode {
    DcMotor motor5;
    DcMotor motor6;

    @Override
    public void init() {
        motor5 = hardwareMap.get(DcMotor.class, "la");
        motor6 = hardwareMap.get(DcMotor.class, "ra");
        telemetry.addData("Do it", "Work?");

    }

    @Override
    public void loop() {
        boolean leftInput = gamepad2.x;
        boolean rightInput = gamepad2.b;
        if (leftInput) {
            motor5.setPower(-1);
            motor6.setPower(-1);
            try {
                Thread.sleep(1000);
            } catch (InterruptedException e) {
                telemetry.addLine("Oops! Something went wrong with positioning the arm!");
            }
        }
        if (rightInput) {
            motor5.setPower(1);
            motor6.setPower(1);
            try {
                Thread.sleep(1000);
            } catch (InterruptedException e) {
                telemetry.addLine("Oops! Something went wrong with positioning the arm!");
            }
        }
        motor5.setPower(0);
        motor6.setPower(0);
    }
}


