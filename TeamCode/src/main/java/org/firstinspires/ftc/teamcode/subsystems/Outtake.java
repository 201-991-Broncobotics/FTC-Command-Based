package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class Outtake extends SubsystemBase {
    CRServo servo;
    public Outtake (HardwareMap map) {
        servo = map.get(CRServo.class, "box2");
    }
    public void shootOut() {
        servo.setPower(1);
        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        servo.setPower(-1);
        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        servo.setPower(0);
    }
    public void setPower(double p) {
        servo.setPower(p);
    }

}