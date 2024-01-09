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
    DistanceSensor dSensor2;
    DistanceSensor dSensor3;
    DistanceSensor dSensor4;
    Telemetry telemetry;
    public double dsResultOne;
    public double dsResultTwo;
    public double dsResultThree;
    public double dsResultFour;

    public DSensor(HardwareMap map) {
        dSensor = map.get(DistanceSensor.class, "dsensorfront");
        dSensor2 = map.get(DistanceSensor.class, "dsensorleft");
        dSensor3 = map.get(DistanceSensor.class, "dsensorright");
        dSensor4 = map.get(DistanceSensor.class, "dsensorback");
    }
    public void readFront(){
        dsResultOne = dSensor.getDistance(DistanceUnit.INCH);
        telemetry.addData("Distance Sensor Read: ", dsResultOne);
    }
    public void readLeft(){
        dsResultTwo = dSensor2.getDistance(DistanceUnit.INCH);
        telemetry.addData("Distance Sensor Read: ", dsResultTwo);
        telemetry.update();
    }
    public void readRight(){
        dsResultThree = dSensor3.getDistance(DistanceUnit.INCH);
        telemetry.addData("Distance Sensor Read: ", dsResultThree);
        telemetry.update();
    }
    public void readBack(){
        dsResultFour = dSensor4.getDistance(DistanceUnit.INCH);
        telemetry.addData("Distance Sensor Read: ", dsResultFour);
        telemetry.update();
    }
    public void turnOff() {
        telemetry.addLine("Distance Sensor Turning Off");
        telemetry.update();
        dSensor.close();
        dSensor2.close();
        dSensor3.close();
        dSensor4.close();
    }
}
