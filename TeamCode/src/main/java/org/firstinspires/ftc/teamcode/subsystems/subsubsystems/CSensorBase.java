package org.firstinspires.ftc.teamcode.subsystems.subsubsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class CSensorBase extends SubsystemBase {
    RevColorSensorV3 colorSensor;
    public void ColorSensor(HardwareMap map) {

        colorSensor = map.get(RevColorSensorV3.class, "CSensor");

    }
    public void CSensorStartUp() {
        colorSensor.initialize();
        if (colorSensor.initialize()) {
            telemetry.addLine("Color Sensor Working!");
            colorSensor.enableLed(true);
        }
        else {
            telemetry.addLine("Color Sensor Not Initalized!");
        }
    }
    public void GetTeamPropDistanceRED() { //Finds the distance the RED Team Prop is from the robot (ideally), use Red alliance code
        if (colorSensor.red() > 1) {
            double Distance = colorSensor.getDistance(DistanceUnit.INCH);
            telemetry.addLine("Red Found!");
            telemetry.addData("Red:", colorSensor.red());
            telemetry.addLine("Blue Is: " + Distance + "Inches Away!");
            telemetry.update();
        }
    }
    public void GetTeamPropDistanceBLUE() { //See above. Same thing but for the BLUE team prop
        if (colorSensor.blue() > 1) {
            double Distance = colorSensor.getDistance(DistanceUnit.INCH);
            telemetry.addLine("Blue Found!");
            telemetry.addData("Blue:", colorSensor.blue());
            telemetry.addLine("Blue Is: " + Distance + "Inches Away!");
            telemetry.update();
        }
    }
    public void CSensorNotActive() {
        colorSensor.close();
    }
}

