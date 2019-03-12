package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Team12167MechanumTeleOp", group="Iterative Opmode")
public class Team12167MechanumTeleOp extends OpMode{
    private DcMotor fr ;
    private DcMotor fl ;
    private DcMotor br ;
    private DcMotor bl ;
    private DcMotor belt = null;
    private Servo leftservo = null;
    private Servo rightservo = null;
    private DigitalChannel touchsensor = null;

    public void init() {
        telemetry.addData("Wait", "DO-NOT-START");
        telemetry.update();
        touchsensor = hardwareMap.get(DigitalChannel.class, "touch");
        fr = hardwareMap.dcMotor.get("fr");
        fl = hardwareMap.dcMotor.get ("fl") ;
        br = hardwareMap.dcMotor.get("br");
        bl = hardwareMap.dcMotor.get ("bl") ;
        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        belt = hardwareMap.dcMotor.get("belt_drive");
        leftservo = hardwareMap.servo.get("left");
        rightservo = hardwareMap.servo.get("right");

        telemetry.addData("Ready", "When you are");
        telemetry.update();
    }

    public void loop() {
        telemetry.addData("state", touchsensor.getState());
        telemetry.update();
        if(!touchsensor.getState() && belt.getPower() < 0) {
            belt.setPower(0);
        }
        if (gamepad1.x) {
            fr.setPower(-.7);
            fl.setPower(.3);
            br.setPower(.3);
            bl.setPower(-.7);
        } else if (gamepad1.b) {
            fr.setPower(.3);
            fl.setPower(-.7);
            br.setPower(-.7);
            bl.setPower(.3);
        } else {
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
            fl.setPower(lefty);
            fr.setPower(righty);
            bl.setPower(lefty);
            br.setPower(righty);
        }

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
