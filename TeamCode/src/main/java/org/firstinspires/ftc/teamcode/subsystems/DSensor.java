package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import java.lang.Math;
/*Waffles video showed FOUR distance Sensors surrounding every part of the robot
In the case we need more distance sensors just add them...
*/

public class DSensor extends SubsystemBase {
    DistanceSensor dSensor;
    DistanceSensor dSensor2;
    DistanceSensor dSensor3;
    DistanceSensor dSensor4;
    Telemetry telemetry;
    public double dsResultOne; //Front
    public double dsResultTwo; //Left
    public double dsResultThree; //Right
    public double dsResultFour; //Back
    public double minDistance;
    public double comparedDSOne;
    public double comparedDSTwo;
    public double comparedDSThree;



    public DSensor(HardwareMap map) {
        dSensor = map.get(DistanceSensor.class, "dsensorfront");
        dSensor2 = map.get(DistanceSensor.class, "dsensorleft");
        dSensor3 = map.get(DistanceSensor.class, "dsensorright");
        dSensor4 = map.get(DistanceSensor.class, "dsensorback");
    }
    public double getDsResultOne(){
        dsResultOne = dSensor.getDistance(DistanceUnit.INCH);
        telemetry.addData("Distance Sensor Read: ", dsResultOne);
        return dsResultOne;
    }
    public double getDsResultTwo(){
        dsResultTwo = dSensor2.getDistance(DistanceUnit.INCH);
        telemetry.addData("Distance Sensor Read: ", dsResultTwo);
        telemetry.update();
        return dsResultTwo;
    }
    public double getDsResultThree(){
        dsResultThree = dSensor3.getDistance(DistanceUnit.INCH);
        telemetry.addData("Distance Sensor Read: ", dsResultThree);
        telemetry.update();
        return dsResultThree;
    }
    public double getDsResultFour(){
        dsResultFour = dSensor4.getDistance(DistanceUnit.INCH);
        telemetry.addData("Distance Sensor Read: ", dsResultFour);
        telemetry.update();
        return dsResultFour;
    }
    public void turnOff() {
        telemetry.addLine("Distance Sensor Turning Off");
        telemetry.update();
        dSensor.close();
        dSensor2.close();
        dSensor3.close();
        dSensor4.close();
    }
    public double getMinDistance(){
        double CalcL1 = Math.min(dsResultOne,dsResultTwo);
        double CalcL2 = Math.min(dsResultThree, dsResultFour);
        minDistance = Math.min(CalcL1,CalcL2);
        return minDistance;
    }
    public double getComparedDSOne(){
        comparedDSOne = Double.compare(minDistance, dsResultOne);
        return comparedDSOne;
    }
    public double getComparedDSTwo(){
        comparedDSTwo = Double.compare(minDistance, dsResultTwo);
        return comparedDSTwo;
    }
    public double getComparedDSThree(){
        comparedDSThree = Double.compare(minDistance,dsResultThree);
        return comparedDSThree;
    }
}
