package org.firstinspires.ftc.teamcode.OpenCv.Autonomous;


import static org.firstinspires.ftc.teamcode.OpenCv.TPDetect.Location.Left;
import static org.firstinspires.ftc.teamcode.OpenCv.TPDetect.Location.Middle;
import static org.firstinspires.ftc.teamcode.OpenCv.TPDetect.Location.Right;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.util.jar.Attributes;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.OpenCv.TPDetect;
import org.firstinspires.ftc.teamcode.OpenCv.TPDetect.Location;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous
public class RedBS extends LinearOpMode {

    OpenCvCamera webcam1;
    private static final int CAMERA_WIDTH = 1280; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 720; // height of wanted camera resolution

    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvWebcam camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam 1"), cameraMonitorViewId);
        TPDetect detector = new TPDetect(telemetry);
        camera.setPipeline(detector);


        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(1280,720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });


        waitForStart();
        switch (detector.location) {
            case Right:
                break;
            case Left:
                break;
            case Middle:
                break;
        }
        while (opModeIsActive()) {
            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
        }
        camera.stopStreaming();
    }
}
