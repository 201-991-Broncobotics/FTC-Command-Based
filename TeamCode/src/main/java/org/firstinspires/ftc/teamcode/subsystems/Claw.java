package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Claw extends SubsystemBase {

    Servo servoClaw;
    public Claw(HardwareMap map) {

        servoClaw = map.get(Servo.class, "ClawServo");
        servoClaw.setPosition(0);
    }
    public void Open() {

        servoClaw.setPosition(0.5);
    }
    public void Close() {

        servoClaw.setPosition(0);
    }
    public void InactiveClaw() {
        servoClaw.setPosition(0);
    }
}

