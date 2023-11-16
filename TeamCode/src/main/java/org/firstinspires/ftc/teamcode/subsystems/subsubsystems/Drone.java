package org.firstinspires.ftc.teamcode.subsystems.subsubsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp
public class Drone extends OpMode {
    public CRServo servo2;
    @Override
    public void init() {
        servo2 = hardwareMap.get(CRServo.class, "drone");
    }
    @Override
    public void loop() {
        if (gamepad2.dpad_left) {

            servo2.setPower(-.25);
            try {
                Thread.sleep(1000);
            } catch (InterruptedException e) {
               telemetry.addData("Oops " , "Took too long!");
            }
            servo2.setPower(.25);
            try {
                Thread.sleep(1000);
            } catch (InterruptedException e) {
                telemetry.addData("Oops " , "Took too long!");
            }
            servo2.setPower(0);

        }
        servo2.setPower(0);
    }

}
