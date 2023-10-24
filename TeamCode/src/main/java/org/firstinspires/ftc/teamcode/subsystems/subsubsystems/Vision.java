package org.firstinspires.ftc.teamcode.subsystems.subsubsystems;


import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import static org.firstinspires.ftc.teamcode.subsystems.subsubsystems.Functions.*;

public class Vision extends Thread {

    private final double fx = 578.272, fy = 578.272, tag_size = 0.166; // tag size in m
    // have to figure out the fx, fy for cameras; cx and cy are just half of the pixel widths

    private final OpenCvWebcam webcam;
    private final AprilTagDetectionPipeline pipeline;
    private final Telemetry telemetry;

    private boolean should_be_running = true;

    public Vision(HardwareMap map, Telemetry telemetry, String webcam_name, double x, double y, double z, double yaw, double pitch, int width, int height) {
        // width and height are pixels; most likely 1280 by 720, aka 720p, but can reduce quality to increase speed

        int cameraMonitorViewId = map.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", map.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(map.get(WebcamName.class, webcam_name), cameraMonitorViewId);

        pipeline = new AprilTagDetectionPipeline(tag_size, fx, fy, width / 2.0, height / 2.0, telemetry);

        webcam.setPipeline(pipeline);

        webcam.setMillisecondsPermissionTimeout(3000); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() { webcam.startStreaming(width, height, OpenCvCameraRotation.UPRIGHT); }

            @Override
            public void onError(int errorCode) { telemetry.addLine("Camera failed to open"); }
        });
        webcam.resumeViewport();
        telemetry.addLine("OpenCV Initialized");
        telemetry.update();

        this.telemetry = telemetry;
    }

    public void stopStreaming() {
        webcam.stopStreaming();
        should_be_running = false;
    }

    public void run() {
        while (should_be_running) {
            telemetry.addData("Frame Count", webcam.getFrameCount());
            telemetry.addData("FPS", "" + round(webcam.getFps(), 2));
            telemetry.addData("Total frame time ms", webcam.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs()); // this might be extremely long
            telemetry.addData("Overhead time ms", webcam.getOverheadTimeMs());
            telemetry.addData("Theoretical max FPS", webcam.getCurrentPipelineMaxFps());
            telemetry.addLine();
            telemetry.addData("April Tag Detected", pipeline.getAprilTag());
            telemetry.update();
        }
    }
}

class AprilTagDetectionPipeline extends OpenCvPipeline {
    private int tag_number = -1;

    public AprilTagDetectionPipeline(double tag_size_meters, double fx, double fy, double cx, double cy, Telemetry telemetry) {
        super();

    }

    @Override
    public Mat processFrame(Mat input) {
        return input;
    }

    public double getAprilTag() {
        return tag_number;
    }
}