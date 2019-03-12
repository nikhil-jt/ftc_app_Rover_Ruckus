package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import static java.lang.Thread.sleep;

@TeleOp(name="Team9511TeleOp", group="Iterative Opmode")
public class Team9511TeleOp extends OpMode{
    private DcMotor left = null;
    private DcMotor right = null;
    private DcMotor leftwheel = null;
    private DcMotor rightwheel = null;
    private Servo jewel;

    public void init() {
        telemetry.addData("Wait", "DO-NOT-START");
        telemetry.update();
//        touchsensor = hardwareMap.touchSensor.get("touch");
        left = hardwareMap.dcMotor.get("left_drive");
        right = hardwareMap.dcMotor.get("right_drive");
//        belt = hardwareMap.dcMotor.get("belt_drive");
        leftwheel = hardwareMap.dcMotor.get("left_wheel");
        rightwheel = hardwareMap.dcMotor.get("right_wheel");
        right.setDirection(DcMotorSimple.Direction.REVERSE);
        rightwheel.setDirection(DcMotorSimple.Direction.REVERSE);
        jewel = hardwareMap.servo.get("jewel");

        telemetry.addData("Ready", "When you are");
        telemetry.update();
    }

    public void loop() {
        double multiplier = 0.24;
        if (gamepad1.right_trigger > 0.1) {
            multiplier *= 2;
        }
        if (gamepad1.left_trigger > 0.1) {
            multiplier *= 2;
        }
        double drive = gamepad1.right_stick_y * multiplier;
        double turn = -gamepad1.right_stick_x * multiplier;
        double lefty = drive + turn;
        double righty = drive - turn;
        double max = Math.max(Math.abs(lefty), Math.abs(righty));
        if (max > 1.0) {
            lefty /= max;
            righty /= max;
        }
        left.setPower(lefty);
        right.setPower(righty);
        if(gamepad1.left_bumper) {
            leftwheel.setPower(0.2);
            rightwheel.setPower(0.2);
        } else if(gamepad1.right_bumper) {
            leftwheel.setPower(-0.2);
            rightwheel.setPower(-0.2);
        } else {
            leftwheel.setPower(0);
            rightwheel.setPower(0);
        }
        if(gamepad1.dpad_left) {
            jewel.setPosition(0.00);
        } else if(gamepad1.dpad_right) {
            jewel.setPosition(1.0);
        } else if(gamepad1.dpad_up) {
            jewel.setPosition(0.5);
        } else if(gamepad1.dpad_down) {
            jewel.setPosition(0.25);
        }
    }
}
