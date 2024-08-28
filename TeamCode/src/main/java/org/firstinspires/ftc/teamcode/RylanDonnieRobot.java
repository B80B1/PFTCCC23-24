package org.firstinspires.ftc.teamcode.OpenCv;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcontroller.external.samples.SensorREV2mDistance;

@TeleOp(name = "RylanDonnieRobot")
public class RylanDonnieRobot extends LinearOpMode {
    private DcMotor leftBack;
    private DcMotor rightBack;
    private DcMotor leftFront;
    private DcMotor rightFront;

    @Override
    public void runOpMode(){
        leftBack = hardwareMap.dcMotor.get("leftBack");
        rightBack = hardwareMap.dcMotor.get("rightBack");
        leftFront = hardwareMap.dcMotor.get("leftFront");
        rightFront = hardwareMap.dcMotor.get("rightFront");

        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);

        waitForStart();

        while (opModeIsActive()) {
            // Drive controls
            double drive = -gamepad1.left_stick_y; // Forward/Backward
            double turn = gamepad1.right_stick_x;  // Rotation
            double strafe = gamepad1.left_stick_x; // Strafe left/right

            // Calculate motor powers for strafing
            double leftFrontPower = drive + turn + strafe;
            double rightFrontPower = drive - turn - strafe;
            double leftBackPower = drive + turn - strafe;
            double rightBackPower = drive - turn + strafe;

            // Normalize the power to make sure no motor power exceeds 1.0
            double maxPower = Math.max(Math.abs(leftFrontPower), Math.max(Math.abs(rightFrontPower),
                    Math.max(Math.abs(leftBackPower), Math.abs(rightBackPower))));
            if (maxPower > 1.0) {
                leftFrontPower /= maxPower;
                rightFrontPower /= maxPower;
                leftBackPower /= maxPower;
                rightBackPower /= maxPower;
            }

            // Set motor powers
            leftFront.setPower(leftFrontPower);
            rightFront.setPower(rightFrontPower);
            leftBack.setPower(leftBackPower);
            rightBack.setPower(rightBackPower);

            // Optional: Log motor power values for debugging
            telemetry.addData("LF Power", leftFrontPower);
            telemetry.addData("RF Power", rightFrontPower);
            telemetry.addData("LB Power", leftBackPower);
            telemetry.addData("RB Power", rightBackPower);
            telemetry.update();
        }
    }
}