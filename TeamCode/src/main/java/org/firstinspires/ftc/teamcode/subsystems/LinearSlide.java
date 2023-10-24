package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LinearSlide extends SubsystemBase {

    private final Telemetry telemetry;

    private final MotorEx leader;
    private final MotorEx[] followers;

    public LinearSlide(HardwareMap map, Telemetry telemetry, String[] motor_names, boolean[] invert_motors) {
        this.telemetry = telemetry;

        leader = new MotorEx(map, motor_names[0]);

        followers = new MotorEx[motor_names.length - 1];
    }
}
