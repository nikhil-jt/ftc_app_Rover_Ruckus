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
//@Disabled
@Autonomous(name = "12167JewelBlueTop", group = "Linear Opmode")
public class JewelBlueTop12167 extends BaseAutonomous {
    public void runOpMode() {
        VuforiaTrackable relicTemplate = initAndWaitForStart();
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
        jewelMove(false, relicTemplate);
        if (vuMark == RelicRecoveryVuMark.UNKNOWN) {
            vuMark = RelicRecoveryVuMark.from(relicTemplate);
        }
        gyroturn(0);
        VuforiaTrackableDefaultListener listener = (VuforiaTrackableDefaultListener) relicTemplate.getListener();
        OpenGLMatrix pose = listener.getPose();
        double angleOffset = 0.0;
//        double distanceFromWall = 17.0;

        if (pose != null) {
            Orientation orientation = Orientation.getOrientation(pose, AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
//            angleOffset  = orientation.firstAngle;
//            distanceFromWall = -(pose.getTranslation().get(2) / 25.4);
//            telemetry.addData("Angle and distance", "%.2f, %.2f", angleOffset, distanceFromWall);
//            telemetry.update();
//            sleep(1000);
        }
        gyroturn(0.2, 0 + angleOffset, 2);
        if (vuMark == RelicRecoveryVuMark.UNKNOWN) {
            vuMark = RelicRecoveryVuMark.from(relicTemplate);
        }
        if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
            telemetry.addData("VuMark", "%s visible", vuMark);
        } else {
            telemetry.addData("Nobody", "here");
        }
        telemetry.update();
        gyroDrive(0.5, -28.75, angleOffset, true);
        sleep(500);
        if (vuMark == RelicRecoveryVuMark.LEFT) {

        } else if (vuMark == RelicRecoveryVuMark.CENTER) {
            gyroDrive(0.7, 7, 90, true);
        } else {//RIGHT or unable to see vuMark
            gyroDrive(0.7, 14, 90, true);
        }
        gyroturn(0.3, 162.5, 20);
        encoderMove(0.7, 4.3, 4.3, 1);
        belt.setPower(-1);
        sleep(1000);
        leftservo.setPosition(0.1); //Open left servo
        rightservo.setPosition(0.9); //Open right servo
        sleep(1000);
        belt.setPower(0);
        sleep(1000);
        encoderMove(0.6, -5, 1);
        encoderMove(0.6, 7, 1);
        encoderMove(0.6, -3, 1);
    }

    private void moveAndPushGlyph(double timeToBoxColumn, double angleOffset) {
        gyroDrive(0.5, -28.75, angleOffset, true);
        sleep(500);
        gyroturn(0.3, 90 + angleOffset, 20);
        gyroDrive(0.7, timeToBoxColumn, 90 + angleOffset, true);
        sleep(500);
        gyroturn(0.3, angleOffset, 20);
        gyroDrive(0.7, 4.0, angleOffset, true);
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
