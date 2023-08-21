package org.firstinspires.ftc.teamcode.opmodes.examples;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Variables;

class SimpleMotor extends SubsystemBase {
    private final MotorEx motor;
    private final Telemetry telem;
    private final String name;

    public SimpleMotor(HardwareMap map, String name, Telemetry telem) {
        motor = new MotorEx(map, name, Motor.GoBILDA.RPM_312);
        motor.setRunMode(Motor.RunMode.RawPower);
        this.telem = telem;
        this.name = name;
    }

    public double getPower() {
        return motor.get();
    }

    public void powerForward() {
        motor.set(0.5);
    }

    public void powerBackward() {
        motor.set(-0.5);
    }

    public void stop() {
        motor.set(0);
    }

    @Override
    public void periodic() {
        if (Variables.teleOp) {
            telem.addLine("\"" + name + "\" motor has current speed " + getPower() + ". ");
        }
    }
}

