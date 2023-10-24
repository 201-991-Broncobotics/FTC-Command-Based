package org.firstinspires.ftc.teamcode.opmodes.utility;

import static org.firstinspires.ftc.teamcode.subsystems.subsubsystems.Functions.round;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Variables;
import org.firstinspires.ftc.teamcode.subsystems.subsubsystems.ServoEx;

class SimpleServo extends SubsystemBase {
    private final ServoEx servo;
    private final Telemetry telem;

    public SimpleServo(HardwareMap map, String name, Telemetry telem) {
        servo = new ServoEx(map, name, 0, 0, 300, 150, 10000, 300, false);
        this.telem = telem;
    }

    public double getPower() {
        return servo.get();
    }

    public void powerForward() {
        servo.set(100);
    }

    public void powerBackward() {
        servo.set(-100);
    }

    public void stop() {
        servo.set(0);
    }

    public void set(double speed) {
        servo.set(speed);
    }

    @Override
    public void periodic() {
        servo.update(); // THIS IS CRITICAL TO CALL DURING PERIODIC() !!!
        if (Variables.teleOp) {
            telem.addLine("servo has current speed " + round(servo.get(), 0) + ". ");
            telem.addLine("servo has current position " + round(servo.getRawPosition(), 2) + ". ");
        }
    }
}