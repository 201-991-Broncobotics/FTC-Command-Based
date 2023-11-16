package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp
public class BoxServo extends OpMode {
    public CRServo servo;
    @Override
    public void init() {
        servo = hardwareMap.get(CRServo.class, "box2");
    }
    @Override
    public void loop() {
        if (gamepad2.a) {

            servo.setPower(1);

            try {
                Thread.sleep(1000);
            } catch (InterruptedException e) {
                telemetry.addData("Oops!" , " Took Too Long!");
            }

            servo.setPower(-1);

            try {
                Thread.sleep(1000);
            } catch (InterruptedException e) {
                telemetry.addData("Oops!" , " Took Too Long");
            }

        }
        servo.setPower(0);
    }

}
