package org.firstinspires.ftc.teamcode.opmodes.teleop;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Just the Arm :skull:")
public class ArmTest extends OpMode {
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
        float leftInput = gamepad1.left_trigger;
        float rightInput = gamepad1.right_trigger;
        if (leftInput > 0){
            motor5.setPower(0.33);
            motor6.setPower(0.33);
        }
        motor5.setPower(0);
        motor6.setPower(0);
        if (rightInput > 0) {
            motor5.setPower(-0.33);
            motor6.setPower(-0.33);
        }
        motor5.setPower(0);
        motor6.setPower(0);
    }
}

