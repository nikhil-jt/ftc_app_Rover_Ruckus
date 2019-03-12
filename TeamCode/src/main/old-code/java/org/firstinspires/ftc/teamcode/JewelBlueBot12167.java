package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;

@Autonomous(name = "12167JewelBlueBot", group = "Linear Opmode")
public class JewelBlueBot12167 extends BaseAutonomous {
    public void runOpMode() {
        VuforiaTrackable relicTemplate = initAndWaitForStart();
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
        jewelMove(false, relicTemplate);
        if (vuMark == RelicRecoveryVuMark.UNKNOWN) {
            vuMark = RelicRecoveryVuMark.from(relicTemplate);
        }
        gyroturn(0.2, 0, 2);

        VuforiaTrackableDefaultListener listener = (VuforiaTrackableDefaultListener) relicTemplate.getListener();
        OpenGLMatrix pose = listener.getPose();
        double angleOffset = 0.0;
        if (pose != null) {
            Orientation orientation = Orientation.getOrientation(pose, AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
            // angleOffset = orientation.firstAngle;
//            double distanceFromWall = -(pose.getTranslation().get(2) / 25.4);
        }
        gyroturn(0.2, 0 + angleOffset, 2);
//        gyroDrive(0.5, 21.5, 0 + angleOffset, false);
        encoderMove(0.5, -41.5, 5);
        if (vuMark == RelicRecoveryVuMark.RIGHT) {
            gyroDrive(0.7, 8.5, -100 + angleOffset, true);
        } else if (vuMark == RelicRecoveryVuMark.CENTER) {
            gyroDrive(0.7, 8.5, -75.75 + angleOffset, true);
        } else {//LEFT or unable to see vuMark
            gyroDrive(0.7, 10, -55 + angleOffset, true);
        }
        belt.setPower(-1);
        sleep(950);
        leftservo.setPosition(0.1); //Open left servo
        rightservo.setPosition(0.9); //Open right servo
        belt.setPower(0);
        gyroturn(0.3, -90 + angleOffset, 20);
        if (vuMark == RelicRecoveryVuMark.RIGHT) {
            encoderMove(0.6, -1, 0, 1);
        } else if (vuMark == RelicRecoveryVuMark.CENTER) {
            encoderMove(0.6, -2, 0, 1);
        } else {//LEFT or unable to see vuMark
            encoderMove(0.6, -3, 0, 1);
        }
        encoderMove(0.6, -1, 1);
        encoderMove(0.6, 5, 1);
        encoderMove(0.6, -4.5, 1);
        telemetry.addData("currangle", gyro.getAngularOrientation().firstAngle);
        telemetry.update();
        sleep(500);
    }
    public void runOpMode1() {
        VuforiaTrackable relicTemplate = initAndWaitForStart();
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
        jewelMove(false, relicTemplate);
        if (vuMark == RelicRecoveryVuMark.UNKNOWN) {
            vuMark = RelicRecoveryVuMark.from(relicTemplate);
        }
        gyroturn(0.2, 0, 20);

        VuforiaTrackableDefaultListener listener = (VuforiaTrackableDefaultListener) relicTemplate.getListener();
        OpenGLMatrix pose = listener.getPose();
        double angleOffset = 0.0;

        if (pose != null) {
            Orientation orientation = Orientation.getOrientation(pose, AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
            angleOffset = orientation.firstAngle;
            telemetry.addData("Angle", "%.2f", angleOffset);
            telemetry.update();
//            sleep(5000);
        }
//        gyroturn(0.2, 0 + angleOffset, 20);
//        gyroturn(0);
        if (vuMark == RelicRecoveryVuMark.UNKNOWN) {
            vuMark = RelicRecoveryVuMark.from(relicTemplate);
        }
        if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
            telemetry.addData("VuMark", "%s visible", vuMark);
        } else {
            telemetry.addData("Nobody", "here");
        }
        telemetry.update();
//TODO adjust 19 - since it is a guess
        if (vuMark == RelicRecoveryVuMark.LEFT) {
            moveAndPushGlyph(22.3 + 10.3);
        } else if (vuMark == RelicRecoveryVuMark.CENTER) {
            moveAndPushGlyph(22.3 + 17.35);
        } else {//LEFT or unable to see vuMark
            moveAndPushGlyph(22.3 + 25.4);
        }
        sleep(1000);
    }

    private void moveAndPushGlyph(double timeToBoxColumn) {
        gyroDrive(0.5, -20, 0, true);
        left.setPower(0.5);
        right.setPower(0.5);
        sleep(1000);
        left.setPower(0);
        right.setPower(0);
//        gyroDrive(0.5, 3.5, 0);
        gyroDrive(0.5, -timeToBoxColumn + 20, 0, true);
        sleep(500);
        gyroturn(0.3, -90, 20);
//        gyroDrive(0.7, timeToBoxColumn, 90 + angleOffset);
//        sleep(500);
//        gyroturn(0.3, angleOffset, 20);
        gyroDrive(0.7, 10, -90, true);
        belt.setPower(-1);
        sleep(1000);
        sleep(1000);
        leftservo.setPosition(0.1); //Open left servo
        rightservo.setPosition(0.9); //Open right servo
        belt.setPower(0);
        encoderMove(0.6, -5, 1);
        encoderMove(0.6, 7, 1);
        encoderMove(0.6, -5, 1);
    }
}
