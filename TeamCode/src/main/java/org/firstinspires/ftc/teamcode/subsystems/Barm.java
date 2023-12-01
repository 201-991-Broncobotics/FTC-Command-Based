package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Barm extends SubsystemBase {
    DcMotor motor10;
    DcMotor motor11;
public Barm (HardwareMap map) {

    motor10 = map.get(DcMotor.class, "la");
    motor11 = map.get(DcMotor.class, "ra");
    motor10.setDirection(DcMotor.Direction.FORWARD);
    motor10.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    motor10.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    motor11.setDirection(DcMotor.Direction.FORWARD);
    motor11.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    motor11.setMode(DcMotor.RunMode.RUN_TO_POSITION);
}
public void setDefault() { //Only use once!
    motor10.setTargetPosition(0);
    motor11.setTargetPosition(0);
}
public void setDown() {
    motor10.setPower(-1);
    motor11.setPower(-1);
    try {
        Thread.sleep(3000);
    } catch (InterruptedException e) {

    }
}
public void upPosition() {
    motor10.setPower(1);
    motor11.setPower(1);
}
}

