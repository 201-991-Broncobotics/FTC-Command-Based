package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.dfrobot.HuskyLens;


public class Huskylens extends SubsystemBase {
    //Andrew Was Here And He likes
    HuskyLens camera;
    public boolean PropFound;
    Telemetry telemetry;
    public double blueLength;
    public double redLength;
    public double tagOneLength;
    public double tagTwoLength;
    public double tagThreeLength;
    public double tagFourLength;
    public double tagFiveLength;

    public Huskylens(HardwareMap map) {
        camera = map.get(HuskyLens.class, "huskylens");
    }

    public void initCamera(boolean isOn) {
        camera.initialize();
        if (isOn) {
            camera.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);
            telemetry.addLine("HuskyLens is Activated!");
            telemetry.update();
        }
    }

    public double getRedLength() { //Always call BEFORE calling lookForRed. Same with lookForBlue
        camera.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);
        HuskyLens.Block[] redProp = camera.blocks(2);
        redLength = redProp.length;
        return redLength;
    }

    public void lookForRed() {
        camera.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);
        telemetry.addData("Length of Red Detected: ", redLength);
        telemetry.update();
        if (redLength <= 5) {
            PropFound = true;
            telemetry.addLine("Team Prop Found!");
            telemetry.update();
        } else if (redLength > 5) {
            PropFound = false;
            telemetry.addLine("Team Prop Not Found!");
            telemetry.update();
        }
    }

    public double getBlueLength() {
        camera.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);
        HuskyLens.Block[] blueProp = camera.blocks(1);
        blueLength = blueProp.length;
        return blueLength;
    }

    public void lookForBlue() {
        camera.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);
        telemetry.addData("Length of Red Detected: ", blueLength);
        telemetry.update();
        if (blueLength <= 5) {
            PropFound = true;
            telemetry.addLine("Team Prop Found!");
            telemetry.update();
        } else if (blueLength > 5) {
            PropFound = false;
            telemetry.addLine("Team Prop Not Found!");
            telemetry.update();
        }
    }

    public double getTagOneLength() {
        camera.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
        HuskyLens.Block[] tagOne = camera.blocks(1);
        tagOneLength = tagOne.length;
        return tagOneLength;

    }
    //ALWAYS CALL AT LEAST ONE OF THESE BEFORE CALLING findTag!!!!!!
    public double getTagTwoLength() {
        camera.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
        HuskyLens.Block[] tagTwo = camera.blocks(1);
        tagTwoLength = tagTwo.length;
        return tagTwoLength;
    }
    public double getTagThreeLength() {
        camera.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
        HuskyLens.Block[] tagThree = camera.blocks(1);
        tagThreeLength = tagThree.length;
        return tagThreeLength;
    }
    public double getTagFourLength() {
        camera.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
        HuskyLens.Block[] tagFour = camera.blocks(1);
        tagFourLength = tagFour.length;
        return tagFourLength;
    }
    public double getTagFiveLength() {
        camera.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
        HuskyLens.Block[] tagFive = camera.blocks(1);
        tagFiveLength = tagFive.length;
        return tagFiveLength;
    }

    public void findTag() {
        camera.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
        if (tagOneLength > 2.5) {
            boolean tagOneFound = true;
            telemetry.addLine("Tag One Found!");
            telemetry.update();
        }
        if (tagTwoLength > 2.5) {
            boolean tagTwoFound = true;
            telemetry.addLine("Tag Two Found!");
            telemetry.update();
        }
        if (tagThreeLength > 2.5) {
            boolean tagThreeFound = true;
            telemetry.addLine("Tag Three Found!");
            telemetry.update();
        }
        if (tagFourLength > 2.5) {
            boolean tagFourFound = true;
            telemetry.addLine("Tag Four Found!");
            telemetry.update();
        }
        if (tagFiveLength > 2.5) {
            boolean tagFiveFound = true;
            telemetry.addLine("Tag One Found!");
            telemetry.update();
        }
    }
}

