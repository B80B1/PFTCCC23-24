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
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class RB extends LinearOpMode {
    public void encoderDrive(double rspeed, double fspeed,
                             double backleftInches, double backrightInches,
                             double frontleftInches, double frontrightInches, double timeoutS) {

        int newbackLeftTarget;
        int newbackRightTarget;
        int newfrontLeftTarget;
        int newfrontRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newbackLeftTarget = backleftDrive.getCurrentPosition() + (int)(backleftInches * COUNTS_PER_INCH);
            newbackRightTarget = backrightDrive.getCurrentPosition() + (int)(backrightInches * COUNTS_PER_INCH);
            newfrontLeftTarget = frontleftDrive.getCurrentPosition() + (int)(frontleftInches * COUNTS_PER_INCH);
            newfrontRightTarget = frontrightDrive.getCurrentPosition() + (int)(frontrightInches * COUNTS_PER_INCH);

            backleftDrive.setTargetPosition(newbackLeftTarget);
            backrightDrive.setTargetPosition(newbackRightTarget);
            frontleftDrive.setTargetPosition(newfrontLeftTarget);
            frontrightDrive.setTargetPosition(newfrontRightTarget);

            // Turn On RUN_TO_POSITION
            backleftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backrightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontleftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontrightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();
            backleftDrive.setPower(Math.abs(rspeed));
            backrightDrive.setPower(Math.abs(rspeed));
            frontleftDrive.setPower(Math.abs(fspeed));
            frontrightDrive.setPower(Math.abs(fspeed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive()  &&
                    (runtime.seconds() < timeoutS)) {

                // Display it for the driver.
               /* telemetry.addData("Running to",  " %7d :%7d  :%7d  :%7d", newfrontLeftTarget,  newfrontRightTarget,
                                                                          newbackLeftTarget,  newbackRightTarget);
                telemetry.addData("Currently at",  " at %7d :%7d  :%7d  :%7d",
                                            frontleftDrive.getCurrentPosition(), frontrightDrive.getCurrentPosition(),
                                            backleftDrive.getCurrentPosition(),  backrightDrive.getCurrentPosition());
                telemetry.update(); */
            }

            // Stop all motion;
            frontleftDrive.setPower(0);
            frontrightDrive.setPower(0);
            backleftDrive.setPower(0);
            backrightDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            frontleftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontrightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backleftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backrightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            sleep(10);   // optional pause after each move.






        }
    }
    private DcMotor         backleftDrive   = null;
    private DcMotor         backrightDrive  = null;
    private DcMotor         frontleftDrive = null;
    private DcMotor         frontrightDrive = null;
    private DcMotor         arm = null;
    private Servo pin;
    private ElapsedTime     runtime = new ElapsedTime();
    static final double     COUNTS_PER_MOTOR_REV    = 537.7 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 3.78 ;     // For figuring circumference
    static final double     SLIDE_DIAMETER_INCHES   = 1.43 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.14159);
    static final double     SLIDE_COUNTS_PER_INCH   =(COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (SLIDE_DIAMETER_INCHES * 3.14159);
    static final double     F_DRIVE_SPEED             = 1;
    static final double     R_DRIVE_SPEED             = 1;
    static final double     TURN_SPEED              = 0.2;
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
