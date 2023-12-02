package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Claw extends SubsystemBase {

    Servo servo3;
    public Claw(HardwareMap map) {

        servo3 = map.get(Servo.class, "ClawServo");
        servo3.setPosition(0);
    }
    public void Open() {

        servo3.setPosition(0.5);
    }
    public void Close() {

        servo3.setPosition(0);
    }
    public void InactiveClaw() {

        servo3.setPosition(0);
    }
}