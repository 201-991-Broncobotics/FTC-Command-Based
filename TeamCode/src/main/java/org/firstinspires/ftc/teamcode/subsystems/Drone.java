package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public class Drone extends SubsystemBase {
    Servo servo2;
    public Drone(HardwareMap map) {
        servo2 = map.get(Servo.class, "drone");
        servo2.setDirection(Servo.Direction.FORWARD);
        servo2.setPosition(0);
    }
    public void notEndgame() {
        servo2.setPosition(0.5);
    }
    public void Endgame() {
        servo2.setPosition(-1);
        try {
            Thread.sleep(250);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        servo2.setPosition(1);
        try {
            Thread.sleep(250);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        servo2.setPosition(0);
    }
}
