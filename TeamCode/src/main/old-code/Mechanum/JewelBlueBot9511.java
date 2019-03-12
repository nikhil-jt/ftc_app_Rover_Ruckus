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

@Autonomous(name = "9511JewelBlueBot", group = "Linear Opmode")
public class JewelBlueBot9511 extends BaseAutonomous2 {
    public void runOpMode() {
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
            angleOffset  = orientation.firstAngle;
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
            moveAndPushGlyph(23 + 9.3);
        } else if (vuMark == RelicRecoveryVuMark.CENTER) {
            moveAndPushGlyph(23 + 17.35);
        } else {//LEFT or unable to see vuMark
            moveAndPushGlyph(23 + 24.8);
        }
        sleep(1000);
    }

    private void moveAndPushGlyph(double timeToBoxColumn) {
        gyroDrive(0.5, -timeToBoxColumn, 0);
        sleep(500);
        gyroturn(0.3, -90, 20);
//        gyroDrive(0.7, timeToBoxColumn, 90 + angleOffset);
//        sleep(500);
//        gyroturn(0.3, angleOffset, 20);
        gyroDrive(0.7, 11, -90);
        leftwheel.setPower(-0.4);
        rightwheel.setPower(-0.4);
        sleep(1000);
        sleep(1000);
        encoderMove(0.6, -5, 1);
        encoderMove(0.6, 7, 1);
        encoderMove(0.6, -5, 1);
        leftwheel.setPower(0);
        rightwheel.setPower(0);
    }
}
