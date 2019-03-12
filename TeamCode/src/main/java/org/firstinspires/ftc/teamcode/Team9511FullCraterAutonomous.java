package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.opencv.core.Rect;
@Disabled
@Autonomous(name = "9511 Full Crater", group = "9511")
public class Team9511FullCraterAutonomous extends BaseAutonomous9511 {
    public static String CENTER = "center";
    public static String LEFT = "left";
    public static String RIGHT = "right";

    @Override
    public void runOpMode() {
        VuforiaLocalizer vuforia = initRobot();
        hang();
        box.setPosition(0.15);
        waitForStart();
        String goldSide = getGoldSide(vuforia);
//        goldSide = LEFT;
//        goldSide = CENTER;
//        goldSide = RIGHT;
//        oldOpMode(goldSide);
//        sleep(2000);
        drop();
        claim(goldSide);
        sample(goldSide);
//        park(goldSide);
    }

    private void sample(String goldSide) {
        gyroTurn(0.35, 5);
        if (goldSide.equals(LEFT)) {
            encoderMove(0.3, 4.5, 10);
            up.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            up.setTargetPosition(0);
            up.setPower(1);
            gyroTurn(0.35, -24.5);
            intake.setPosition(0.55);
            encoderMove(0.3, 34, 10);
        } else if (goldSide.equals(CENTER)) {
            encoderMove(0.3, 2, 10);
            up.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            up.setTargetPosition(0);
            up.setPower(1);
            intake.setPosition(0.55);
            encoderMove(0.3, 18, 10);
        } else {
            encoderMove(0.3, 4.5, 10);
            up.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            up.setTargetPosition(0);
            up.setPower(1);
            gyroTurn(0.35, 26);
            intake.setPosition(0.55);
            encoderMove(0.3, 34, 10);
        }
        intakebox.setPosition(0.4);
    }

    private void park(String goldSide) {
        if (goldSide.equals(LEFT)) {
            gyroTurn(0.35, 55);
            encoderMove(0.3, -60, 10);
        } else if (goldSide.equals(CENTER)) {
            gyroTurn(0.35, -90);
            encoderMove(0.3, 12, 10);
            gyroTurn(0.35, -113);
            encoderMove(0.3, 10, 8);
            gyroTurn(0.35, -135);
            encoderMove(0.3, 35, 10);
        } else {
            gyroTurn(0.35, -90);
            encoderMove(0.3, 20, 10);
            gyroTurn(0.35, -110);
            encoderMove(0.3, 13, 10);
            gyroTurn(0.35, -135);
            encoderMove(0.3, 36, 10);
        }
        intake.setPosition(0.6);
        goup();
    }

    private void claim(String goldSide) {
        gyroTurn(0.35, 0.0);
        encoderMove(0.3, 15, 10);
        up.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        up.setTargetPosition(0);
        up.setPower(1);
        gyroTurn(0.3, -74.3);
        encoderMove(0.3, 35, 10);
        gyroTurn(0.35, -118);
        encoderMove(0.3, 20, 10);
        gyroTurn(0.35, -124);
        encoderMove(0.3, 15, 10);
        intakebox.setPosition(0.3);
        sleep(500);
        intake.setPosition(0.04);
        sleep(500);
        intake.setPosition(0.5);
        intakebox.setPosition(0.8);
        encoderMove(0.3, -15, 10);
        gyroTurn(0.35, -118);
        encoderMove(0.3, -20, 10);
        gyroTurn(0.35, -74.3);
        encoderMove(0.3, -10, 10);


    }

    private String getGoldSide(VuforiaLocalizer vuforia) {
        int cameraWidth = getCameraWidth(vuforia);
        telemetry.addData("Width", cameraWidth);
        Rect goldLocation = locateGold(getMatFromVuforia(vuforia));
        if (goldLocation == null) {
            telemetry.addData("Not Found Gold, defaulting to", 0);
            goldLocation = new Rect(cameraWidth/4, 0, 10, 10);
        } else {
//            goldLocation = new Rect(cameraWidth*3/4, 0, 10, 10);
            telemetry.addData("Found Gold at", goldLocation);
        }
        telemetry.update();

        String gold;
        if (goldLocation.x < cameraWidth / 3) {
            gold = RIGHT;
        } else if (goldLocation.x < 2 * cameraWidth / 3) {
            gold = CENTER;
        } else {
            gold = LEFT;
        }
        return gold;
    }

    private void drop() {
        scoringarm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        scoringarm.setPower(0);
        up.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        up.setTargetPosition(-7875);
        up.setPower(1);
        while (up.isBusy() && opModeIsActive()) {
            telemetry.addData("Up motor at", up.getCurrentPosition());
            telemetry.update();
        }
    }
    private void goup() {
        up.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        up.setTargetPosition(0);
        up.setPower(1);
        while (up.isBusy() && opModeIsActive()) {
            telemetry.addData("Up motor at", up.getCurrentPosition());
            telemetry.update();
        }
    }

    private void hang() {
        up.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        up.setTargetPosition(up.getCurrentPosition());
        up.setPower(1);
    }

    private void oldOpMode(String goldSide) {
        if (opModeIsActive()) {
            double goldTurn = 0;
            double goldDist = 0;
            if (goldSide.equals(LEFT)) {
                goldTurn = -24.5;
                goldDist = 31;
                telemetry.addData("Found Gold at", "Left");
                encoderMove(0.2, 5.5, 10);
                gyroTurn(0.35, goldTurn);
                encoderMove(0.3, (goldDist/3)-2, 10);
                intake.setPosition(0.55);
                encoderMove(0.3, goldDist+2, 10);

//            todo collect gold and move to depot
                gyroTurn(0.3, 25);
                encoderMove(0.3, goldDist*5/6, 10);
//                gyroTurn(0.5, 90);
//                encoderMove(0.3, 15, 10);
//            todo deposit team marker
                intake.setPosition(0.6);
                encoderMove(0.3, -5, 10);
//                encoderMove(0.3, -3, 10);
                sleep(500);
                //belt.setPosition(0.5);
                intake.setPosition(0);
//           todo move to crater
                encoderMove(0.3, 3, 10);
                gyroTurn(0.5, -125);
                encoderMove(0.6, 75, 20);
                //todo left
            } else if (goldSide.equals(CENTER)) {
                goldDist = 28;
                telemetry.addData("Found Gold at", "Center");
                encoderMove(0.2, 3, 10);
                gyroTurn(0.35, 0);
                encoderMove(0.3, (goldDist/3)-8, 10);
                intake.setPosition(0.55);
                encoderMove(0.3, 2*goldDist/3, 10);

//            todo collect gold and move to depot
                gyroTurn(0.5, 0);
                encoderMove(0.3, goldDist, 10);
                gyroTurn(0.35, 0);
                encoderMove(0.3, 15, 10);
//            todo deposit team marker
                intake.setPosition(0.6);
                encoderMove(0.3, -3, 10);
                sleep(500);
                //belt.setPosition(0.5);
                intake.setPosition(0);
//           todo move to crater
                encoderMove(0.3, 3, 10);
                gyroTurn(0.5, -110);
                encoderMove(0.4, 10, 20);
                gyroTurn(0.5, -130);
                encoderMove(0.6, 56, 20);
                gyroTurn(0.5, -140);
//            todo park
                encoderMove(0.6, 15, 10);
                //todo center
            } else {
                //todo doto
                goldTurn = 24.5;
                goldDist = 31;
                telemetry.addData("Found Gold at", "Right");
                encoderMove(0.2, 5.5, 10);
                gyroTurn(0.35, -2 + goldTurn);
                encoderMove(0.3, (goldDist/3)-2, 10);
                intake.setPosition(0.55);
                encoderMove(0.3, goldDist+2, 10);

//            todo collect gold and move to depot
                gyroTurn(0.30, -20);
                encoderMove(0.3, goldDist*5/6, 10);
//                gyroTurn(0.5, 90);
//                encoderMove(0.3, 15, 10);
//            todo deposit team marker
                intake.setPosition(0.6);
                encoderMove(0.3, -5, 10);
//                encoderMove(0.3, -3, 10);
                sleep(500);
                //belt.setPosition(0.5);
                intake.setPosition(0);
//           todo move to crater
                encoderMove(0.3, 3, 10);
                gyroTurn(0.5, -100);
                encoderMove(0.4, 20, 20);
                gyroTurn(0.5, -110);
                encoderMove(0.4, 9, 20);
                gyroTurn(0.5, -130);
                encoderMove(0.6, 63, 20);
            }
            telemetry.update();
//            sleep(200);
////            todo move to where goldLocation is
//            encoderMove(0.3, 5.5, 10);
//            gyroTurn(0.5, 90 + goldTurn);
//            encoderMove(0.3, goldDist/3, 10);
//            intake.setPosition(0.28);
//            encoderMove(0.3, 2*goldDist/3, 10);
//
////            todo collect gold and move to depot
//            gyroTurn(0.5, 90 -goldTurn);
//            encoderMove(0.3, goldDist, 10);
//            gyroTurn(0.5, 90);
//            encoderMove(0.3, 15, 10);
////            todo deposit team marker
//            //belt.setPosition(1);
//            encoderMove(0.3, -3, 10);
//            sleep(500);
//            //belt.setPosition(0.5);
//            intake.setPosition(0);
////           todo move to crater
//            encoderMove(0.3, 3, 10);
//            gyroTurn(0.5, 200);
//            encoderMove(0.4, 10, 20);
//            gyroTurn(0.5, 220);
//            encoderMove(0.4, 56, 20);
//            gyroTurn(0.5, 230);
////            todo park
//            encoderMove(0.4, 15, 10);
        }

    }
}
