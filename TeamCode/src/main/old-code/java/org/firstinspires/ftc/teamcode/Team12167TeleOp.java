package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import static java.lang.Thread.sleep;

@TeleOp(name="Team12167TeleOp", group="Iterative Opmode")
public class Team12167TeleOp extends OpMode{
    private DcMotor left = null;
    private DcMotor right = null;
    private DcMotor belt = null;
    private Servo leftservo = null;
    private Servo rightservo = null;
    private DigitalChannel touchsensor = null;

    public void init() {
        telemetry.addData("Wait", "DO-NOT-START");
        telemetry.update();
        touchsensor = hardwareMap.get(DigitalChannel.class, "touch");
        touchsensor.setMode(DigitalChannel.Mode.INPUT);
        left = hardwareMap.dcMotor.get("left_drive");
        right = hardwareMap.dcMotor.get("right_drive");
        belt = hardwareMap.dcMotor.get("belt_drive");
        leftservo = hardwareMap.servo.get("left");
        rightservo = hardwareMap.servo.get("right");
        right.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Ready", "When you are");
        telemetry.update();
    }

    public void loop() {
        telemetry.addData("state", touchsensor.getState());
        telemetry.update();
        if(!touchsensor.getState() && belt.getPower() < 0) {
            belt.setPower(0);
        }
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
            belt.setPower(1);
        } else if(gamepad1.right_bumper) {
            if(touchsensor.getState()) {
                belt.setPower(-1);
            } else {
                belt.setPower(0);
            }
        } else {
            belt.setPower(0);
        }
        if(gamepad1.dpad_left) {
            leftservo.setPosition(0.05);
            rightservo.setPosition(0.9);
        } else if(gamepad1.dpad_right) {
            leftservo.setPosition(0.45);
            rightservo.setPosition(0.55);
        } else if(gamepad1.dpad_up) {
            leftservo.setPosition(0.2);
            rightservo.setPosition(0.8);
        } else if(gamepad1.dpad_down) {
            leftservo.setPosition(0.35);
            rightservo.setPosition(0.65);
        }
//        if(gamepad1.a) {
//            try {
//
//                moveDown();
//            } catch (InterruptedException e) {
//                e.printStackTrace();
//            }
//        }
    }
}
