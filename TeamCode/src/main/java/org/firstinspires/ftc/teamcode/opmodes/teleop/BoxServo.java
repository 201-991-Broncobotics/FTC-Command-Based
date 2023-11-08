package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp
public class BoxServo extends OpMode {
    public CRServo servo;
    @Override
    public void init() {
        servo = hardwareMap.get(CRServo.class, "box");
    }
    @Override
    public void loop() {
        while (gamepad1.x) {

            servo.setPower(1);

            try {
                Thread.sleep(2500);
            } catch (InterruptedException e) {
                telemetry.addData("Oops!" , " Took Too Long!");
            }

            servo.setPower(-1);

            try {
                Thread.sleep(2500);
            } catch (InterruptedException e) {
                telemetry.addData("Oops!" , " Took Too Long");
            }

        }
        servo.setPower(0);
    }

}
