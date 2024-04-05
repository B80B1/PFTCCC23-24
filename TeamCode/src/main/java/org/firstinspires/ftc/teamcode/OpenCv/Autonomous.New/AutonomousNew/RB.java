package org.firstinspires.ftc.teamcode.OpenCv.Autonomous.New.AutonomousNew;

import static org.firstinspires.ftc.teamcode.OpenCv.Detectors.TPDetectR.Location.Left;
import static org.firstinspires.ftc.teamcode.OpenCv.Detectors.TPDetectR.Location.Right;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.tuning.MecanumDrive;
import org.firstinspires.ftc.teamcode.OpenCv.Detectors.TPDetectR;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous
public class RB extends LinearOpMode {

    OpenCvCamera webcam1;
    private static final int CAMERA_WIDTH = 1280; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 720; // height of wanted camera resolution


        public void runOpMode() throws InterruptedException {
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            OpenCvWebcam camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam 1"), cameraMonitorViewId);
            TPDetectR detector = new TPDetectR(telemetry);
            camera.setPipeline(detector);

            camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                @Override
                public void onOpened() {
                    camera.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
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
                if (detector.location == Left) {

                } else if (detector.location == Right) {

                } else {

                }
            }
        }
    }
