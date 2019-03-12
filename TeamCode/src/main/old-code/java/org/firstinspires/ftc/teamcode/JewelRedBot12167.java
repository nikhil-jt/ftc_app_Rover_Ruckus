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

@Autonomous(name = "12167JewelRedBot", group = "Linear Opmode")
public class JewelRedBot12167 extends BaseAutonomous {
    @Override
    public void runOpMode() {
        VuforiaTrackable relicTemplate = initAndWaitForStart();
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
        jewelMove(true, relicTemplate);
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
        encoderMove(0.5, 21.5, 5);

        if (vuMark == RelicRecoveryVuMark.RIGHT) {
            gyroDrive(0.7, 8.5, -80 + angleOffset, true);
        } else if (vuMark == RelicRecoveryVuMark.CENTER) {
            gyroDrive(0.7, 8.5, -60 + angleOffset, true);
        } else {//LEFT or unable to see vuMark
            gyroDrive(0.7, 11.5, -40 + angleOffset, true);
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
        encoderMove(0.6, -5, 2);
        encoderMove(0.6, 7, 2);
        encoderMove(0.6, -13, 3);
        gyroturn(0.6, 90, 1);
        telemetry.addData("currangle", gyro.getAngularOrientation().firstAngle);
        telemetry.update();
        sleep(500);
    }

    public void runOpMode1() {
        VuforiaTrackable relicTemplate = initAndWaitForStart();
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
        jewelMove(true, relicTemplate);
        if (vuMark == RelicRecoveryVuMark.UNKNOWN) {
            vuMark = RelicRecoveryVuMark.from(relicTemplate);
        }
        gyroturn(0.2, 0, 20);

        VuforiaTrackableDefaultListener listener = (VuforiaTrackableDefaultListener) relicTemplate.getListener();
        OpenGLMatrix pose = listener.getPose();
        double angleOffset = 0.0;

        if (pose != null) {
            Orientation orientation = Orientation.getOrientation(pose, AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
            angleOffset  = orientation.firstAngle;
            telemetry.addData("Angle", "%.2f", angleOffset);
            telemetry.update();
        }
        gyroturn(0.2, 0 + angleOffset, 20);
        if (vuMark == RelicRecoveryVuMark.UNKNOWN) {
            vuMark = RelicRecoveryVuMark.from(relicTemplate);
        }
        if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
            telemetry.addData("VuMark", "%s visible", vuMark);
        } else {
            telemetry.addData("Nobody", "here");
        }
        telemetry.update();
        if (vuMark == RelicRecoveryVuMark.RIGHT) {
            moveAndPushGlyph(20 + 3.5, angleOffset);
        } else if (vuMark == RelicRecoveryVuMark.CENTER) {
            moveAndPushGlyph(20 + 10.9, angleOffset);
        } else {//LEFT or unable to see vuMark
            moveAndPushGlyph(20 + 18, angleOffset);
        }
        sleep(1000);
    }

    private void moveAndPushGlyph(double timeToBoxColumn, double angleOffset) {
        gyroDrive(0.5, timeToBoxColumn, 0 + angleOffset, false);
        sleep(500);
        gyroturn(0.3, -90 + angleOffset, 20);
        gyroDrive(0.7, 9.5, -90 + angleOffset, true);
        belt.setPower(-1);
        sleep(1000);
        sleep(1000);
        belt.setPower(0);
        leftservo.setPosition(0.1); //Open left servo
        rightservo.setPosition(0.9); //Open right servo
        encoderMove(0.6, -5, 1);
        encoderMove(0.6, 7, 1);
        encoderMove(0.6, -5, 1);
    }
}
