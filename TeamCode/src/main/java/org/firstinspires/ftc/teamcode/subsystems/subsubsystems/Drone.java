package org.firstinspires.ftc.teamcode.subsystems.subsubsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@TeleOp
public class Drone extends OpMode {
    public CRServo servo2;
    @Override
    public void init() {
        servo2 = hardwareMap.get(CRServo.class, "drone");
    }
    @Override
    public void loop() {

        boolean DPADPressed = false;

        while (DPADPressed = false) {
            servo2.setPower(-.25);

        }
            servo2.setPower(0);
        if (gamepad2.dpad_left) {

            DPADPressed = true;
            servo2.setPower(1);
            try {
                Thread.sleep(250);
            } catch (InterruptedException e) {
               telemetry.addData("Oops " , "Took too long!");
            }
            servo2.setPower(-1);
            try {
                Thread.sleep(250);
            } catch (InterruptedException e) {
                telemetry.addData("Oops " , "Took too long!");
            }
            servo2.setPower(0);

        }
        servo2.setPower(0);
    }

}
