package org.firstinspires.ftc.teamcode.OpenCv;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "RylanDonnieRobot")
public class RylanDonnieRobot extends LinearOpMode {
    private DcMotor leftBack;
    private DcMotor rightBack;
    private DcMotor leftFront;
    private DcMotor rightFront;

    @Override
    public void runOpMode() {
        // Initialize hardware
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
            // Get joystick values
            double forwardMotion = -gamepad1.left_stick_y; // Invert to match expected control
            double strafeMotion = gamepad1.left_stick_x;
            double rotateMotion = gamepad1.right_stick_x; // Rotate based on right stick

            // Calculate motor power
            double leftFrontPower = forwardMotion + strafeMotion - rotateMotion;
            double rightFrontPower = forwardMotion - strafeMotion + rotateMotion;
            double leftBackPower = forwardMotion - strafeMotion - rotateMotion;
            double rightBackPower = forwardMotion + strafeMotion + rotateMotion;

            // Normalize the motor powers
            double maxPower = Math.max(Math.abs(leftFrontPower),
                    Math.max(Math.abs(rightFrontPower),
                            Math.max(Math.abs(leftBackPower),
                                    Math.abs(rightBackPower))));

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
        }