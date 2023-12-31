package org.firstinspires.ftc.teamcode.subsystems.subsubsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;
//It's not gonna work stop coping - Mael
public class CSensor extends SubsystemBase {
    public double Distance;
    public boolean ColorFound;
    public boolean ColorFound2;
    RevColorSensorV3 colorSensor;

    public CSensor(HardwareMap map) {

        colorSensor = map.get(RevColorSensorV3.class, "CSensor");
    }
    public void CSensorStartUp() {
        colorSensor.initialize();
            colorSensor.enableLed(true);
        }
    public void GetTeamPropDistanceRED() { //Finds the distance the RED Team Prop is from the robot (ideally), use Red alliance code
        colorSensor.getRawLightDetected();
        if (colorSensor.red() > 1) {
            ColorFound = true;
        }
        else if (colorSensor.red() < 0.1){
            ColorFound = false;
        }
    }
    public void GetTeamPropDistanceRED2() {
        colorSensor.getLightDetected();
        if (colorSensor.red() > 1) {
            ColorFound2 = true;
        }
        else if (colorSensor.red() < 0.1){
            ColorFound2 = false;
        }
    }
    public void GetTeamPropDistanceBLUE() { //See above. Same thing but for the BLUE team prop
        colorSensor.getLightDetected();
        if (colorSensor.blue() > 1) {
            ColorFound = true;
        }
        else if (colorSensor.blue() < 0.1){
            ColorFound = false;
        }
    }
    public void GetTeamPropDistanceBLUE2() {
        colorSensor.getLightDetected();
        if (colorSensor.blue() > 1) {
            ColorFound2 = true;
        } else if (colorSensor.blue() < 0.1) {
            ColorFound2 = false;
        }
    }
    public void CSensorNotActive() {
        colorSensor.close();
    }
}

