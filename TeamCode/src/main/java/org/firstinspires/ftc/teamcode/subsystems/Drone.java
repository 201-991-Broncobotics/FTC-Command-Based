package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.CRServo;


public class Drone extends SubsystemBase {
    CRServo servo2;
    public Drone(HardwareMap map) {
        servo2 = map.get(CRServo.class, "drone");
        servo2.setDirection(CRServo.Direction.FORWARD);
    }
    public void holup() {
        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }
    public void notEndgame() {
        servo2.setPower(0);
    }
    public void Endgame() {
        servo2.setPower(-1);
        holup();
        servo2.setPower(1);
        holup();
    }
}
