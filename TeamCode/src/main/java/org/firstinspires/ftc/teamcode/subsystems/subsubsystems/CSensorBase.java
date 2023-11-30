package org.firstinspires.ftc.teamcode.subsystems.subsubsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class CSensorBase extends SubsystemBase {
    public double Distance;
    public boolean ColorFound;
    public boolean ColorFound2;
    RevColorSensorV3 colorSensor;

    public CSensorBase (HardwareMap map) {

        colorSensor = map.get(RevColorSensorV3.class, "CSensor");
    }
    public void CSensorStartUp() {
        colorSensor.initialize();
        if (colorSensor.initialize()) {
            telemetry.addLine("Color Sensor Working!");
            colorSensor.enableLed(true);
        }
        else {
            telemetry.addLine("Color Sensor Not Initialized!");
        }
    }
    public void GetTeamPropDistanceRED() { //Finds the distance the RED Team Prop is from the robot (ideally), use Red alliance code
        colorSensor.getRawLightDetected();
        if (colorSensor.red() > 1) {
            ColorFound = true;
            Distance = colorSensor.getDistance(DistanceUnit.INCH);
            telemetry.addLine("Red Found!");
            telemetry.addData("Red: ", colorSensor.red());
            telemetry.addLine("Blue Is: " + Distance + " Inches Away!");
            telemetry.update();
        }
        else if (colorSensor.red() < 0.1){
            ColorFound = false;
            telemetry.addLine("Red Not Found!");
        }
    }
    public void GetTeamPropDistanceRED2() {
        colorSensor.getLightDetected();
        if (colorSensor.red() > 1) {
            ColorFound2 = true;
            Distance = colorSensor.getDistance(DistanceUnit.INCH);
            telemetry.addLine("Red Found!");
            telemetry.addData("Red: ", colorSensor.red());
            telemetry.addLine("Blue Is: " + Distance + " Inches Away!");
            telemetry.update();
        }
        else if (colorSensor.red() < 0.1){
            ColorFound2 = false;
            telemetry.addLine("Red Not Found!");
        }
    }
    public void GetTeamPropDistanceBLUE() { //See above. Same thing but for the BLUE team prop
        colorSensor.getLightDetected();
        if (colorSensor.blue() > 1) {
            ColorFound = true;
            Distance = colorSensor.getDistance(DistanceUnit.INCH);
            telemetry.addLine("Blue Found!");
            telemetry.addData("Blue: ", colorSensor.blue());
            telemetry.addLine("Blue Is: " + Distance + " Inches Away!");
            telemetry.update();
        }
        else if (colorSensor.blue() < 0.1){
            ColorFound = false;
            telemetry.addLine("Blue Not Found!");
        }
    }
    public void GetTeamPropDistanceBLUE2() {
        colorSensor.getLightDetected();
        if (colorSensor.blue() > 1) {
            ColorFound2 = true;
            Distance = colorSensor.getDistance(DistanceUnit.INCH);
            telemetry.addLine("Blue Found!");
            telemetry.addData("Blue: ", colorSensor.blue());
            telemetry.addLine("Blue Is: " + Distance + " Inches Away!");
            telemetry.update();
        } else if (colorSensor.blue() < 0.1) {
            ColorFound2 = false;
            telemetry.addLine("Blue Not Found!");
        }
    }
    public void CSensorNotActive() {
        colorSensor.close();
    }
}

