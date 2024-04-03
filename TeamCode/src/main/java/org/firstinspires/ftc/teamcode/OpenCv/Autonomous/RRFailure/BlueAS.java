package org.firstinspires.ftc.teamcode.OpenCv.Autonomous.RRFailure;


import static java.lang.Math.toRadians;

import androidx.annotation.NonNull;

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

    private static class Arm {
        private DcMotor Arm;

        public Arm (HardwareMap hardwareMap) {
            Arm = hardwareMap.get(DcMotor.class, "arm");
            Arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            Arm.setDirection(DcMotor.Direction.FORWARD);
        }

        public class ArmUp implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    Arm.setPower(0.8);
                    initialized = true;
                }

                double pos = Arm.getCurrentPosition();
                packet.put("ArmPos", pos);
                if (pos < 3000.0) {
                    return true;
                } else {
                    Arm.setPower(0);
                    return false;
                }
            }
        }
        public Action ArmUp() {
            return new ArmUp();
        }

        public class ArmDown implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    Arm.setPower(-0.8);
                    initialized = true;
                }

                double pos = Arm.getCurrentPosition();
                packet.put("ArmPos", pos);
                if (pos > 100.0) {
                    return true;
                } else {
                    Arm.setPower(0);
                    return false;
                }
            }
        }
        public Action ArmDown(){
            return new ArmDown();
        }
    }

    private class Pin {
        private Servo Pin;

        public Pin(HardwareMap hardwareMap) {
            Pin = hardwareMap.get(Servo.class, "Intake");
            }
        public class ClosePin implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                Pin.setPosition(0.55);
                return false;
            }
        }
        public Action ClosePin() {
            return new ClosePin();
        }

        public class OpenPin implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                Pin.setPosition(1.0);
                return false;
            }
        }
        public Action OpenPin() {
            return new OpenPin();
        }
    }



        @Override
        public void runOpMode() throws InterruptedException {
            MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(-60, -35, Math.toRadians(270)));
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            OpenCvWebcam camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam 1"), cameraMonitorViewId);
            TPDetectB detector = new TPDetectB(telemetry);
            camera.setPipeline(detector);
            Pin pin = new Pin(hardwareMap);
            Arm arm = new Arm(hardwareMap);

            Action trajectoryActionL1;
            Action trajectoryActionR1;
            Action trajectoryActionM1;
            Action trajectoryActionL2;
            Action trajectoryActionR2;
            Action trajectoryActionM2;
            Action trajectoryEnd;

            trajectoryActionL1 = drive.actionBuilder(drive.pose)
                    .splineTo(new Vector2d(-27.5, -40), Math.toRadians(0))
                    .waitSeconds(1)
                    .splineTo(new Vector2d(-10, -35), Math.toRadians(0))
                    .lineToY(15)
                    .build();

            trajectoryActionL2 = drive.actionBuilder(drive.pose)
                    .splineToConstantHeading(new Vector2d(-42.5, 55), Math.toRadians(0))
                    .build();

            trajectoryActionR1 = drive.actionBuilder(drive.pose)
                    .splineTo(new Vector2d(-32.5, -32.5), Math.toRadians(180))
                    .waitSeconds(1)
                    .splineTo(new Vector2d(-10, -35), Math.toRadians(0))
                    .lineToY(15)
                    .build();

            trajectoryActionR2 = drive.actionBuilder(drive.pose)
                    .splineToConstantHeading(new Vector2d(-32.5, 55), Math.toRadians(0))
                    .build();

            trajectoryActionM1 = drive.actionBuilder(drive.pose)
                    .splineTo(new Vector2d(-35, -32.5), Math.toRadians(270))
                    .waitSeconds(1)
                    .splineTo(new Vector2d(-10, -35), Math.toRadians(0))
                    .lineToY(15)
                    .build();

            trajectoryActionM2 = drive.actionBuilder(drive.pose)
                    .splineToConstantHeading(new Vector2d(-32.5, 55), Math.toRadians(0))
                    .build();

            trajectoryEnd = drive.actionBuilder(drive.pose)
                    .splineTo(new Vector2d(-60, 40), Math.toRadians(180))
                    //.splineTo(new Vector2d(-10, 40), Math.toRadians(180))
                    .lineToY(60)
                    .build();

            Actions.runBlocking(pin.ClosePin());

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

                Action trajectoryChosenAction1;
                Action trajectoryChosenAction2;
                if (detector.location == TPDetectB.Location.Left) {
                        trajectoryChosenAction1 = trajectoryActionL1;
                        trajectoryChosenAction2 = trajectoryActionL2;
                } else if (detector.location == TPDetectB.Location.Right) {
                        trajectoryChosenAction1 = trajectoryActionR1;
                        trajectoryChosenAction2 = trajectoryActionR2;
                } else {
                        trajectoryChosenAction1 = trajectoryActionM1;
                        trajectoryChosenAction2 = trajectoryActionM2;

                }
                Actions.runBlocking(
                        new SequentialAction(
                                trajectoryChosenAction1,
                                arm.ArmUp(),
                                pin.OpenPin(),
                                arm.ArmDown(),
                                trajectoryChosenAction2,
                                trajectoryEnd
                        )
                );
            }

            camera.stopStreaming();
        }
    }

