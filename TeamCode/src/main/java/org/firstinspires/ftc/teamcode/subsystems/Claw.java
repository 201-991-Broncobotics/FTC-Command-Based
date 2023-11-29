package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Claw extends SubsystemBase {

    DcMotor motor8;
    Servo servo3;
    public Claw(HardwareMap map) {

        motor8 = map.get(DcMotor.class, "ClawDC");
        motor8.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor8.setDirection(DcMotor.Direction.FORWARD);
        motor8.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        servo3 = map.get(Servo.class, "ClawServo");
        servo3.setPosition(0);
    }
    public void Open() {

        servo3.setPosition(0.5);
    }
    public void Close() {

        servo3.setPosition(0);
    }
    public void MoveUp() {

        motor8.setPower(0.5);
    }
    public void MoveDown() {

        motor8.setPower(-0.5);
    }
    public void InactiveClaw() {

        servo3.setPosition(0);
    }
    public void InactiveArm() {

        motor8.setPower(0);
    }

}
