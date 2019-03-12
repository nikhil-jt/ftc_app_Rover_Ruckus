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

import static java.lang.Math.abs;

@Autonomous(name = "12167JewelRedTop", group = "Linear Opmode")
public class JewelRedTop12167 extends BaseAutonomous {
    @Override
    public void runOpMode() {
        VuforiaTrackable relicTemplate = initAndWaitForStart();

        jewelMove(true, relicTemplate);
        gyroturn(0.2, 0, 2);

        VuforiaTrackableDefaultListener listener = (VuforiaTrackableDefaultListener) relicTemplate.getListener();
        OpenGLMatrix pose = listener.getPose();
        double angleOffset = 0.0;

        if (pose != null) {
            Orientation orientation = Orientation.getOrientation(pose, AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
            angleOffset  = orientation.firstAngle;
//            double distanceFromWall = -(pose.getTranslation().get(2) / 25.4);
        }
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
        gyroturn(0.2, 0 + angleOffset, 2);
        telemetry.addData("angles", "%.2f, %.2f", angleOffset, gyro.getAngularOrientation().firstAngle);
        telemetry.update();
        sleep(2000);
//        gyroDrive(0.5, 21.5, 0 + angleOffset, false);
        encoderMove(0.5, 21.5, 5);

        if (vuMark == RelicRecoveryVuMark.RIGHT) {
            gyroDrive(0.7, 8.5, 15 + angleOffset, true);
        } else if (vuMark == RelicRecoveryVuMark.CENTER) {
            gyroDrive(0.7, 8.5, 30 + angleOffset, true);
        } else {//LEFT or unable to see vuMark
            gyroDrive(0.7, 11.5, 55 + angleOffset, true);
        }

        belt.setPower(-1);
        sleep(950);
        leftservo.setPosition(0.3); //Open left servo
        rightservo.setPosition(0.7); //Open right servo
        belt.setPower(0);
        encoderMove(0.6, 3, -2, 1);
        encoderMove(0.6, -5, 1);
        leftservo.setPosition(0.1);
        rightservo.setPosition(0.9);
        encoderMove(0.6, 6, 1);
        gyroturn(0.6, 0, 1);
        encoderMove(0.6, -5, 1);
        gyroturn(0.6, 135, 20);
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
        double distanceFromWall = 14.0;

        if (pose != null) {
            Orientation orientation = Orientation.getOrientation(pose, AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
            angleOffset  = orientation.firstAngle;
            distanceFromWall = -(pose.getTranslation().get(2) / 25.4);
            telemetry.addData("Angle and distance", "%.2f, %.2f", angleOffset, distanceFromWall);
            telemetry.update();
//            sleep(1000);
        }
        gyroturn(0.2, 0 + angleOffset, 20);
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
        if (vuMark == RelicRecoveryVuMark.RIGHT) {
            moveAndPushGlyph(20 - distanceFromWall, angleOffset);
        } else if (vuMark == RelicRecoveryVuMark.CENTER) {
            moveAndPushGlyph(20 + 7.0 - distanceFromWall, angleOffset);
        } else {//LEFT or unable to see vuMark
            moveAndPushGlyph(20 + 12.5 - distanceFromWall, angleOffset);
        }
        sleep(1000);
    }

    private void moveAndPushGlyph(double timeToBoxColumn, double angleOffset) {
        gyroDrive(0.5, 22.0, angleOffset, true);
        sleep(500);
        gyroturn(0.3, 90 + angleOffset, 20);
        gyroDrive(0.7, timeToBoxColumn, 90 + angleOffset, true);
        sleep(500);
        gyroturn(0.3, angleOffset, 20);
        gyroDrive(0.7, 4.5, angleOffset, true);
        belt.setPower(-1);
        sleep(1000);
        leftservo.setPosition(0.1); //Open left servo
        rightservo.setPosition(0.9); //Open right servo
        sleep(1000);
        belt.setPower(0);
        encoderMove(0.6, -5, 1);
        encoderMove(0.6, 7, 1);
        encoderMove(0.6, -5, 1);
    }
}
