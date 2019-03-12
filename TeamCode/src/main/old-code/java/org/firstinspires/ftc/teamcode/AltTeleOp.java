package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import static java.lang.Thread.sleep;

/**
 * Created by jitto on 06/10/17.
 */
@TeleOp(name="AltTeleOp", group="Iterative Opmode")

public class AltTeleOp extends OpMode{
    DcMotor left = null;
    DcMotor right = null;
    DcMotor belt = null;
    Servo leftservo = null;
    Servo rightservo = null;
    Servo jewelservo = null;
    double jewelpos = 0.3;

    public void init() {
        left = hardwareMap.dcMotor.get("left_drive");
        right = hardwareMap.dcMotor.get("right_drive");
        belt = hardwareMap.dcMotor.get("belt_drive");
        leftservo = hardwareMap.servo.get("left");
        rightservo = hardwareMap.servo.get("right");
        jewelservo = hardwareMap.servo.get("jewel");
        jewelservo.setPosition(jewelpos);
        left.setDirection(DcMotorSimple.Direction.REVERSE);
//        leftservo.setPosition(1.0);
//        rightservo.setPosition(0.0);
//        sleepMs(1000);
//        leftservo.setPosition(0.45);
//        rightservo.setPosition(0.5);
//        sleepMs(1000);
//        leftservo.setPosition(0.8);
//        rightservo.setPosition(0.2);
//        sleepMs(1000);
//        leftservo.setPosition(0.7);
//        rightservo.setPosition(0.3);
//        sleepMs(1000);
//        leftservo.setPosition(0.6);
//        rightservo.setPosition(0.4);
//        sleepMs(1000);
//        leftservo.setPosition(0.5);
//        rightservo.setPosition(0.5);
    }

    private void sleepMs(int milliSeconds) {
        try {
            sleep(milliSeconds);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    public void loop() {
        left.setPower(gamepad1.left_stick_y);
        right.setPower(gamepad1.right_stick_y);
        if(gamepad1.left_trigger > 0.1) {
            belt.setPower(gamepad1.left_trigger);
//            jewelservo.setPosition(gamepad1.left_trigger);
//            telemetry.addData("Power", gamepad1.left_trigger);
        } else if(gamepad1.right_trigger > 0.1) {
            belt.setPower(-gamepad1.right_trigger);
//            jewelservo.setPosition(-gamepad1.right_trigger);
//            telemetry.addData("Power", -gamepad1.right_trigger);
        } else {
            belt.setPower(0);
//            jewelservo.setPosition(0);
//            telemetry.addData("Power", 0);
        }
        if(gamepad1.left_bumper) {
            leftservo.setPosition(1);
            rightservo.setPosition(0);
        } else if(gamepad1.right_bumper) {
            leftservo.setPosition(0.45);
            rightservo.setPosition(0.5);
        }
        jewelservo.setPosition(jewelpos);
    }
}
