package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "TeleOpTutorial")
public class TeleOpTutorial extends LinearOpMode {
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor rightBack;

    private Servo bluearm;
    private Servo orangearm;

    @Override
    public void runOpMode() throws InterruptedException {
        leftFront = hardwareMap.dcMotor.get("leftFront");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        leftBack = hardwareMap.dcMotor.get("leftBack");
        rightBack = hardwareMap.dcMotor.get("rightBack");
        bluearm = hardwareMap.servo.get("bluearm");
        orangearm = hardwareMap.servo.get("orangearm");


        leftBack.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {

            leftFront.setPower(-gamepad1.left_stick_y);
            rightFront.setPower(-gamepad1.right_stick_y);
            leftBack.setPower(-gamepad1.left_stick_y);
            rightBack.setPower(-gamepad1.right_stick_y);

            if (gamepad1.right_bumper) {
                bluearm.setPosition(0.5);
            } else {
                bluearm.setPosition(0.0);
                if (gamepad1.right_bumper) {
                    orangearm.setPosition(-0.5);
                } else {
                    orangearm.setPosition(0.0);
                    idle();
                }
            }
        }
    }
}