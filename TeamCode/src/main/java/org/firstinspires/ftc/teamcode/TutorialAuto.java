package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "TutorialAuto")
public class TutorialAuto extends LinearOpMode{

    private DcMotor leftBack;
    private DcMotor rightBack;
    private DcMotor slide;

    private Servo bluearm;
    private Servo orangearm;

    @Override
    public void runOpMode() throws InterruptedException {
        leftBack = hardwareMap.dcMotor.get("leftBack");
        rightBack = hardwareMap.dcMotor.get("rightBack");
        slide = hardwareMap.dcMotor.get("slide");
        bluearm = hardwareMap.servo.get("bluearm");
        orangearm = hardwareMap.servo.get("orangearm");


        leftBack.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        while(opModeIsActive()) {
            leftBack.setPower(1);
            rightBack.setPower(1);
            sleep(450);
            leftBack.setPower(1);
            rightBack.setPower(-1);
            sleep(450);
            leftBack.setPower(1);
            rightBack.setPower(1);
            sleep(450);
            leftBack.setPower(1);
            rightBack.setPower(-1);
            sleep(450);
            leftBack.setPower(1);
            rightBack.setPower(1);
            sleep(450);
            leftBack.setPower(1);
            rightBack.setPower(-1);
            sleep(450);
            leftBack.setPower(1);
            rightBack.setPower(1);
            sleep(450);
            leftBack.setPower(1);
            rightBack.setPower(-1);
            sleep(450);
        }


    }
}