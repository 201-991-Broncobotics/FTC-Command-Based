package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.dfrobot.HuskyLens;


public class Huskylens extends SubsystemBase {

    HuskyLens camera;
    public boolean PropFound = false;
    Telemetry telemetry;

    public Huskylens(HardwareMap map) {
        camera = map.get(HuskyLens.class, "huskylens");
    }
    public void initCamera(boolean isOn){
        camera.initialize();
        if (isOn){
            camera.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);
            telemetry.addLine("HuskyLens is Activated!");
            telemetry.update();
        }
    }
    public void lookForRed(){
        HuskyLens.Block[] redProp = camera.blocks(2);
        double redLength = redProp.length;
        telemetry.addData("Length of Red Detected: ", redLength);
        telemetry.update();
        if (redLength < 5){
            PropFound = true;
            telemetry.addLine("Team Prop Found!");
            telemetry.update();
        }
        else if (redLength > 10){
            PropFound = false;
            telemetry.addLine("Team Prop Not Found!");
            telemetry.update();
        }
    }
    public void lookForBlue(){
        HuskyLens.Block[] blueProp = camera.blocks(1);
        double blueLength = blueProp.length;
        telemetry.addData("Length of Red Detected: ", blueLength);
        telemetry.update();
        if (blueLength < 5){
            PropFound = true;
            telemetry.addLine("Team Prop Found!");
            telemetry.update();
        }
        else if (blueLength > 10){
            PropFound = false;
            telemetry.addLine("Team Prop Not Found!");
            telemetry.update();
        }
    }
    public void findTag(double tagOneLength, double tagTwoLength, double tagThreeLength,
                        double tagFourLength, double tagFiveLength, boolean tagOneFound,
                        boolean tagTwoFound, boolean tagThreeFound, boolean tagFourFound,
                        boolean tagFiveFound){
        camera.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
        HuskyLens.Block[] tagOne = camera.blocks(3);
        HuskyLens.Block[] tagTwo = camera.blocks(4);
        HuskyLens.Block[] tagThree = camera.blocks(5);
        HuskyLens.Block[] tagFour = camera.blocks(6);
        HuskyLens.Block[] tagFive = camera.blocks(7);
        tagOneLength = tagOne.length;
        tagTwoLength = tagTwo.length;
        tagThreeLength = tagThree.length;
        tagFourLength = tagFour.length;
        tagFiveLength = tagFive.length;
        if (tagOneLength > 2){
            tagOneFound = true;
            telemetry.addLine("Tag One Found!");
            telemetry.update();
        }
        if (tagTwoLength > 2){
            tagTwoFound = true;
            telemetry.addLine("Tag Two Found!");
            telemetry.update();
        }
        if (tagThreeLength > 2){
            tagThreeFound = true;
            telemetry.addLine("Tag Three Found!");
            telemetry.update();
        }
        if (tagFourLength > 2){
            tagFourFound = true;
            telemetry.addLine("Tag Four Found!");
            telemetry.update();
        }
        if (tagFiveLength > 2){
            tagFiveFound = true;
            telemetry.addLine("Tag One Found!");
            telemetry.update();
        }
    }
}
