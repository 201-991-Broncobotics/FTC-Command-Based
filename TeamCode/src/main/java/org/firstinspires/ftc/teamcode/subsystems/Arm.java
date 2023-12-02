package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class Arm extends SubsystemBase {
    DcMotor motor5;
    DcMotor motor6;

    public Arm (HardwareMap map) {
        motor5 = map.get(DcMotor.class, "la");
        motor6 = map.get(DcMotor.class, "ra");

        motor5.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor5.setDirection(DcMotor.Direction.FORWARD);
        motor5.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor6.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor6.setDirection(DcMotor.Direction.FORWARD);
        motor6.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }
    public void up() {
        motor5.setPower(-1);
        motor6.setPower(-1);
    }
    public void down() {
        motor5.setPower(1);
        motor6.setPower(1);
    }
    public void InactiveArm(){
        motor5.setPower(0);
        motor6.setPower(0);
    }
    public void setPower(double p) {
        motor5.setPower(p);
        motor6.setPower(p);
    }

}
