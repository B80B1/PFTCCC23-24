package org.firstinspires.ftc.teamcode.rookies;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class TeleOpForVoid extends OpMode {
    DcMotor Backleft = null;
    DcMotor Backright =null;
      public void init() {
          Backleft = hardwareMap.get(DcMotor.class, "BL");
          Backright = hardwareMap.get(DcMotor.class, "BR");

          Backleft.setDirection(DcMotorSimple.Direction.FORWARD);
          Backright.setDirection(DcMotorSimple.Direction.FORWARD);
      }

          public void loop () {
              if (gamepad1.left_stick_y != 0.0) {
                   Backleft.setPower(-gamepad1.left_stick_y);
                   Backright.setPower(-gamepad1.right_stick_y);

               }
              else{
                  Backleft.setPower(0.0);
                  Backright.setPower(0.0);
              }
          }
      }