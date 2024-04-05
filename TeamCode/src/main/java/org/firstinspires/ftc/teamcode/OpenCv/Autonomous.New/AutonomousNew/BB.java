package org.firstinspires.ftc.teamcode.OpenCv.Autonomous.New.AutonomousNew;

import static org.firstinspires.ftc.teamcode.OpenCv.Detectors.TPDetectB.Location.Left;
import static org.firstinspires.ftc.teamcode.OpenCv.Detectors.TPDetectB.Location.Right;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.OpenCv.Detectors.TPDetectB;
import org.firstinspires.ftc.teamcode.tuning.MecanumDrive;
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
public class BB extends LinearOpMode {

    OpenCvCamera webcam1;
    private static final int CAMERA_WIDTH = 1280; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 720; // height of wanted camera resolution

    public static class Arm {
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
            return ArmUp();
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
        public static Action ArmDown(){
            return ArmDown();
        }
    }

    public class Pin {
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
            return new Pin.ClosePin();
        }

        public class OpenPin implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                Pin.setPosition(1.0);
                return false;
            }
        }
        public Action OpenPin() {
            return new Pin.OpenPin();
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(60, 12.5, Math.toRadians(90)));
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
                .lineToY(40)
                .waitSeconds(1)
                .build();

        trajectoryActionL2 = drive.actionBuilder(drive.pose)
                .splineTo(new Vector2d(25, 55), Math.toRadians(0))
                .build();

        trajectoryActionR1 = drive.actionBuilder(drive.pose)
                .splineTo(new Vector2d(30, 7.5), Math.toRadians(0))
                .waitSeconds(1)
                .build();

        trajectoryActionR2 = drive.actionBuilder(drive.pose)
                .splineTo(new Vector2d(35, 55), Math.toRadians(0))
                .build();

        trajectoryActionM1 = drive.actionBuilder(drive.pose)
                .splineTo(new Vector2d(32.5, 15), Math.toRadians(90))
                .waitSeconds(1)
                .build();

        trajectoryActionM2 = drive.actionBuilder(drive.pose)
                .splineTo(new Vector2d(40, 55), Math.toRadians(0))
                .build();

        trajectoryEnd = drive.actionBuilder(drive.pose)
                .lineToYSplineHeading(40, Math.toRadians(180))
                //.splineTo(new Vector2d(10, 40), Math.toRadians(180))
                .lineToY(60)
                .build();

        Actions.runBlocking(pin.ClosePin());

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
        Action trajectoryChosenAction1;
        Action trajectoryChosenAction2;
        if (detector.location == Left) {
            trajectoryChosenAction1 = trajectoryActionL1;
            trajectoryChosenAction2 = trajectoryActionL2;
        } else if (detector.location == Right) {
            trajectoryChosenAction1 = trajectoryActionR1;
            trajectoryChosenAction2 = trajectoryActionR2;
        } else {
            trajectoryChosenAction1 = trajectoryActionM1;
            trajectoryChosenAction2 = trajectoryActionM2;
        }
        while (opModeIsActive()) {

            telemetry.addData("Detector If Statement", detector.location);
            telemetry.update();

            Actions.runBlocking(
                    new SequentialAction(
                            trajectoryChosenAction1,
                            trajectoryEnd));
        }
    }
}
