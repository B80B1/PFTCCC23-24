package org.firstinspires.ftc.teamcode.OpenCv.Autonomous;


import static java.lang.Math.toRadians;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.OpenCv.TPDetectB;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.ParallelAction;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous
public class BlueAS extends LinearOpMode {

    OpenCvCamera webcam1;

    private static final int CAMERA_WIDTH = 1280; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 720; // height of wanted camera resolution

    public class Arm {
        private DcMotor Arm;

        public Arm(HardwareMap hardwareMap) {
            Arm = hardwareMap.get(DcMotor.class, "arm");
        }
    }

    public class Pin {
        private Servo Pin;

        public Pin(HardwareMap hardwareMap) {
            Pin = hardwareMap.get(Servo.class, "Intake");
            }
        }

        @Override
        public void runOpMode() throws InterruptedException {
            MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(-60, -35, Math.toRadians(270)));
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            OpenCvWebcam camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam 1"), cameraMonitorViewId);
            TPDetectB detector = new TPDetectB(telemetry);
            camera.setPipeline(detector);

            Action trajectoryActionL;
            Action trajectoryActionR;
            Action trajectoryActionM;
            Action trajectoryEnd;

            trajectoryActionL = drive.actionBuilder(drive.pose)
                    .splineTo(new Vector2d(-27.5, -40), Math.toRadians(0))
                    .waitSeconds(1)
                    .splineTo(new Vector2d(-10, -35), Math.toRadians(0))
                    .lineToY(15)
                    .splineToConstantHeading(new Vector2d(-42.5, 55), Math.toRadians(0))
                    .build();

            trajectoryActionR = drive.actionBuilder(drive.pose)
                    .splineTo(new Vector2d(-32.5, -32.5), Math.toRadians(0))
                    .waitSeconds(1)
                    .splineTo(new Vector2d(-10, -35), Math.toRadians(0))
                    .lineToY(15)
                    .splineToConstantHeading(new Vector2d(-32.5, 55), Math.toRadians(0))
                    .build();

            trajectoryActionM = drive.actionBuilder(drive.pose)
                    .splineTo(new Vector2d(-35, -32.5), Math.toRadians(0))
                    .waitSeconds(1)
                    .splineTo(new Vector2d(-10, -35), Math.toRadians(0))
                    .lineToY(15)
                    .splineToConstantHeading(new Vector2d(-32.5, 55), Math.toRadians(0))
                    .build();
            trajectoryEnd = drive.actionBuilder(drive.pose)
                    .splineTo(new Vector2d(-60, 40), Math.toRadians(180))
                    //.splineTo(new Vector2d(-10, 40), Math.toRadians(180))
                    .lineToY(60)
                    .build();

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
                // Don't burn CPU cycles busy-looping in this sample

                Action trajectoryChosenAction;
                if (detector.location == TPDetectB.Location.Left) {
                        trajectoryChosenAction = trajectoryActionL;
                } else if (detector.location == TPDetectB.Location.Right) {
                        trajectoryChosenAction = trajectoryActionR;
                } else {
                        trajectoryChosenAction = trajectoryActionM;
                }
                Actions.runBlocking(
                        new SequentialAction(
                                trajectoryChosenAction,
                                trajectoryEnd
                        )
                );
            }

            camera.stopStreaming();
        }
    }

