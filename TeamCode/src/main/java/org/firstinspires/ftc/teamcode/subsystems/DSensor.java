package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.Telemetry;
/*Waffles video showed FOUR distance Sensors surrounding every part of the robot
In the case we need more distance sensors just add them...
*/

public class DSensor extends SubsystemBase {
    DistanceSensor dSensor;
    Telemetry telemetry;
    public double dsResult;

    public DSensor(HardwareMap map) {
        map.get(DistanceSensor.class, "dsensor");
    }
    public void read() {
        dsResult = dSensor.getDistance(DistanceUnit.INCH);
        telemetry.addData("Distance Sensor Read: ", dsResult);
    }
    public void turnOff() {
        telemetry.addLine("Distance Sensor Turning Off");
        dSensor.close();
    }
}
