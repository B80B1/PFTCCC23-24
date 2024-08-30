package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp(name = "TeleOpLearn", group = "Tutorial")
public class TelOpTutorial extends LinearOpMode
{
    private DcMotor LeftBack;
    private DcMotor LeftFront;
    private DcMotor RightBack;
    private DcMotor RightFront;
    @Override
public void runOpMode () throws InterruptedException {
        LeftBack = hardwareMap.dcMotor.get("motorLeftRear");
        LeftFront = hardwareMap.dcMotor.get("motorLeftFront");
        RightBack = hardwareMap.dcMotor.get("motorRightRear");
        RightFront = hardwareMap.dcMotor.get("motorRightFront");

        LeftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        LeftFront.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive())

            LeftBack.setPower(-gamepad1.left_stick_y);
      LeftFront.setPower(-gamepad1.left_stick_y);
        RightBack.setPower(-gamepad1.right_stick_y);
        RightFront.setPower(-gamepad1.right_stick_y);

        idle();

    }
    }








