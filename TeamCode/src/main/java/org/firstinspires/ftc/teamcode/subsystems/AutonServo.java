package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
public class AutonServo extends SubsystemBase {

    CRServo autonservo;
    Telemetry telemetry;

    public AutonServo(HardwareMap map) {

        autonservo = map.get(CRServo.class, "autonservo");
        autonservo.setRunMode(Motor.RunMode.RawPower);
        autonservo.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
    }
    public void notAuton(){
        autonservo.stopMotor();
    }
    public void duringAuton() {
        for (int i = 0; i < 4; i++) {
            autonservo.set(.1);
            try {
                Thread.sleep(3000);
            } catch (InterruptedException e) {
                telemetry.addLine("Auton Servo Messed Up");
                telemetry.update();
            }
            autonservo.stopMotor();
        }
    }
}
