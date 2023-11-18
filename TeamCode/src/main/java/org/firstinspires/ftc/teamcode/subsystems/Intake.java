package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
public class Intake extends SubsystemBase {

    DcMotor motor7;

    public Intake (HardwareMap map) {
        motor7 = map.get(DcMotor.class, "lintake");
        motor7.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor7.setDirection(DcMotor.Direction.FORWARD);
        motor7.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }
    public void suck() {
        motor7.setPower(-0.5);
    }
    public void blow() {
        motor7.setPower(0.5);
    }
    public void InactiveIntake() {
        motor7.setPower(0);
    }
    public void setPower(double p) {
        motor7.setPower(p);
    }

}
