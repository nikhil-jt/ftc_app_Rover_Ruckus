package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import static java.lang.Math.max;
import static java.lang.Math.min;

@TeleOp(name = "Team9511DualScoringTeleOp", group = "Iterative Opmode")
public class Team9511DualScoringTeleOp extends OpMode {
    private DcMotorEx scoringarm;
    private Servo intake;
    private Servo box;
    private Servo intakebox;
    private DcMotor right;
    private DcMotor left;
    private DcMotor linear;
    private DcMotor up;
    public double boxv = 0.87;
    public Boolean b = false;
    public Boolean lastpressed = false;
    private ScoringState scoringState = ScoringState.DOWN;
    private double targetRuntime;
    public int basecount = 0;
    public int limit = -7875;

    @Override
    public void init() {
        telemetry.addData("Status", "Initializing");
        right = hardwareMap.dcMotor.get("br");
        up = hardwareMap.dcMotor.get("up");
        linear = hardwareMap.dcMotor.get("linear");
        linear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        up.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        up.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        up.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left = hardwareMap.dcMotor.get("bl");
        right.setDirection(DcMotorSimple.Direction.REVERSE);
        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakebox = hardwareMap.servo.get("intakebox");
        intake = hardwareMap.servo.get("intake");
        scoringarm = (DcMotorEx) hardwareMap.dcMotor.get("scoring");
        box = hardwareMap.servo.get("box");
        scoringarm.setDirection(DcMotorSimple.Direction.REVERSE);
        scoringarm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        basecount = scoringarm.getCurrentPosition();
        scoringarm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        scoringarm.setPower(0);
        scoringarm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        box.setPosition(0);
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        if (gamepad1.dpad_left && scoringState == ScoringState.DOWN) {
            basecount = scoringarm.getCurrentPosition();
            scoringarm.setVelocity(100, AngleUnit.DEGREES);
            scoringarm.setTargetPosition(410 + basecount);
            scoringState = ScoringState.GOING_UP;
        }
        if(scoringState ==  ScoringState.DOWN) {
            box.setPosition(0.13);
        }
        if (scoringState == ScoringState.GOING_UP) {
            double diff = (410 + basecount) - scoringarm.getCurrentPosition();
            scoringarm.setVelocity(max(min(diff / 4, 100), 60), AngleUnit.DEGREES);
            box.setPosition(min(max(((double) (scoringarm.getCurrentPosition() - basecount)) / 500, 0.0), 0.7));
            if (!scoringarm.isBusy()) {
                scoringState = ScoringState.MOVE_FORWARD;
                scoringarm.setVelocity(20, AngleUnit.DEGREES);
                scoringarm.setTargetPosition(500 + basecount);
                box.setPosition(0.7);
            }
        }
        if (scoringState == ScoringState.MOVE_FORWARD) {
            if (!scoringarm.isBusy()) {
                targetRuntime = getRuntime() + 0.5;
                scoringarm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                scoringarm.setPower(0);
                scoringState = ScoringState.MOVE_FORWARD_STABLE;
            }

        }
        if (scoringState == ScoringState.MOVE_FORWARD_STABLE && getRuntime() > targetRuntime) {
            box.setPosition(0.3);
            targetRuntime = getRuntime() + 1.5;
            scoringState = ScoringState.ATTEMPT_1_DOWN;
        }
        if (scoringState == ScoringState.ATTEMPT_1_DOWN && getRuntime() > targetRuntime) {
            box.setPosition(0.5);
            targetRuntime = getRuntime() + .5;
            scoringState = ScoringState.ATTEMPT_1_UP;
        }
        if (scoringState == ScoringState.ATTEMPT_1_UP && getRuntime() > targetRuntime) {
            box.setPosition(0.3);
            targetRuntime = getRuntime() + .5;
            scoringState = ScoringState.ATTEMPT_2_DOWN;
        }
        if (scoringState == ScoringState.ATTEMPT_2_DOWN && getRuntime() > targetRuntime) {
            box.setPosition(0.5);
            targetRuntime = getRuntime() + .5;
            scoringState = ScoringState.ATTEMPT_2_UP;
        }
        if (scoringState == ScoringState.ATTEMPT_2_UP && getRuntime() > targetRuntime) {
            scoringarm.setVelocity(20, AngleUnit.DEGREES);
            scoringarm.setTargetPosition(basecount);
            scoringState = ScoringState.GOING_DOWN;
        }
        if (scoringState == ScoringState.GOING_DOWN) {
            if (scoringarm.getCurrentPosition() < basecount + 20) {
                scoringarm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                scoringarm.setPower(0);
                scoringState = ScoringState.DOWN;
            }
            box.setPosition(min(max(((double) (scoringarm.getCurrentPosition() - basecount)) / 900, 0.0), 0.5));
        }
        telemetry.addData("ScoringArm", "Target=%d, Current=%d, box=%.2f, state=%s", scoringarm.getTargetPosition(), scoringarm.getCurrentPosition(), box.getPosition(), scoringState);
        float x = gamepad1.right_stick_x;
        float y = gamepad1.right_stick_y;
        b = gamepad1.b ? (!lastpressed ? !b : b) : b;
        lastpressed = gamepad1.b;
        y = b ? -y : y;
        double multiplier = gamepad1.left_trigger > 0.1 ? 1 : 0.5;
        multiplier = gamepad1.right_trigger > 0.1 ? 0.25 : multiplier;
        double leftp = (y + x) * multiplier;
        double rightp = (y - x) * multiplier;
        left.setPower(rightp);
        right.setPower(leftp);
        if (gamepad2.dpad_up) {
            boxv += 0.01;
        }
        if (gamepad2.dpad_down) {
            boxv -= 0.01;
        }
        if (gamepad2.dpad_right) {
            boxv = 0.47;
        }
        if (gamepad2.dpad_left) {
            boxv = 0.8;
        }
        boxv = Range.clip(boxv, 0.3, 0.87);
        intakebox.setPosition(boxv);
        intake.setPosition(gamepad2.left_bumper ? 0.04 : (gamepad2.right_bumper ? 0.94 : 0.5));
        telemetry.addData("intake", intake.getPosition());
        if (gamepad1.y) {
            if (up.getCurrentPosition() < limit && !gamepad1.x) {
                telemetry.addData("height", "max");
                up.setPower(0);
            } else {
                if (up.getCurrentPosition() < limit + 600) {
                    up.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    up.setPower(-0.3);
                } else {
                    up.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    up.setPower(-1);
                }
//                up.setPower(up.getCurrentPosition() < limit + 400 ? Range.clip(-1 * (400 - (up.getCurrentPosition() - (limit + 400))) / 400, -1, -0.2) : -1);
            }
        } else if (gamepad1.a) {
            if (up.getCurrentPosition() > 0 && !gamepad1.x) {
                telemetry.addData("height", "max");
                up.setPower(0);
            } else {
                up.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                up.setPower(up.getCurrentPosition() > -400 ? Range.clip((400 - (up.getCurrentPosition() + 400)) / 400, 0.4, 1) : 1);
            }
        } else {
                up.setPower(0);
        }
        if(gamepad2.x) {
            up.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            up.setTargetPosition(-4120);
            up.setPower(0.8);
        }
//        if(gamepad1.left_stick_x < -0.8) {
//            linear.setPower(1);
//            linear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            linear.setTargetPosition(-85);
//        }
//        if(gamepad1.left_stick_x > 0.8) {
//            linear.setPower(1);
//            linear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            linear.setTargetPosition(-435);
//        }
//        if(linear.getMode() == DcMotor.RunMode.RUN_TO_POSITION && !linear.isBusy()) {
//            linear.setPower(0);
//            linear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        }
//        if(gamepad1.left_stick_y > 0 && linear.getCurrentPosition() > -435 && linear.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
//            linear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            linear.setPower(gamepad1.left_stick_y);
//        }
//        if(gamepad1.left_stick_y < 0 && linear.getCurrentPosition() < 5 && linear.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
//            linear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        }
        linear.setPower(gamepad2.right_stick_y);
        telemetry.addData("x: ", x);
        telemetry.addData("y: ", y);
        telemetry.addData("upcount", up.getCurrentPosition());
        telemetry.addData("linear", linear.getCurrentPosition());
        telemetry.addData("box", boxv);
        telemetry.addData("scorebox", box.getPosition());
        telemetry.addData("arm", scoringarm.getCurrentPosition());

    }

    enum ScoringState {
        DOWN, GOING_UP, MOVE_FORWARD, MOVE_FORWARD_STABLE, ATTEMPT_1_DOWN, ATTEMPT_1_UP, ATTEMPT_2_DOWN, ATTEMPT_2_UP, GOING_DOWN;
    }
}