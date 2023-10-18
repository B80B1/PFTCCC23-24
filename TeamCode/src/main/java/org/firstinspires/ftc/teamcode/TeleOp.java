package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Tele-Op")

public class TeleOp extends LinearOpMode {

    private DcMotor motor1 = null;
    private DcMotor motor2 = null;
    private DcMotor motor3 = null;
    private DcMotor motor4 = null;
    private DcMotor motor5 = null; //slide control right
    private DcMotor motor6 = null; //slide control left
    // private CRServo CRR = null;
    // private CRServo CRL = null;

    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        motor1 = hardwareMap.get(DcMotor.class,"leftFront"); //frontleft, port 0
        motor2 = hardwareMap.get(DcMotor.class,"rightFront");  //frontright, port 1
        motor3 = hardwareMap.get(DcMotor.class,"leftBack"); //backleft, port 3
        motor4 = hardwareMap.get(DcMotor.class,"rightBack");  //backright, port 2
        motor5 = hardwareMap.get(DcMotor.class, "arm");
        motor6 = hardwareMap.get(DcMotor.class, "arm t");
        // CRR = hardwareMap.get(CRServo.class, "CR1");
        // CRL = hardwareMap.get(CRServo.class, "CR2");


        motor1.setDirection(DcMotor.Direction.FORWARD);
        motor2.setDirection(DcMotor.Direction.REVERSE);
        motor3.setDirection(DcMotor.Direction.FORWARD);
        motor4.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {

            double forwardMotion;
            double horizonMotion;
            double rotateMotion;

            forwardMotion = gamepad1.right_stick_y;
            horizonMotion = gamepad1.right_stick_x;
            rotateMotion = gamepad1.left_stick_x;

            double denominator = Math.max(Math.abs(forwardMotion) + Math.abs(horizonMotion) + Math.abs(rotateMotion), 1);
            double m1Power = (forwardMotion + horizonMotion + rotateMotion) / denominator;
            double m2Power = (forwardMotion + horizonMotion - rotateMotion) / denominator;
            double m3Power = (forwardMotion - horizonMotion + rotateMotion) / denominator;
            double m4Power = (forwardMotion - horizonMotion - rotateMotion) / denominator;
            double m5Power = gamepad2.right_stick_y;
            double m6Power = gamepad2.right_stick_x;

            motor1.setPower(m1Power);
            motor2.setPower(m2Power);
            motor3.setPower(m3Power);
            motor4.setPower(m4Power);
            motor5.setPower(m5Power);
            motor6.setPower(m6Power);

            telemetry.addData("m1", m1Power);
            telemetry.addData("m2", m2Power);
            telemetry.addData("m3", m3Power);
            telemetry.addData("m4", m4Power);
            telemetry.addData("m5", m5Power);
            telemetry.addData("m6", m6Power);
            telemetry.addData("FM", forwardMotion);
            telemetry.addData("HM", horizonMotion);
            telemetry.addData("RM", rotateMotion);
            telemetry.addData("Status", "Running");
            telemetry.update();

        }
    }
}
