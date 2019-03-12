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

@Autonomous(name = "TestAutonomous", group = "Linear Opmode")
public class TestAutonomous extends BaseAutonomous {
    @Override
    public void runOpMode() {
        VuforiaTrackable relicTemplate = initAndWaitForStart();
        left.setPower(0.7);
        left2.setPower(0.7);
        right.setPower(0.7);
        right2.setPower(0.7);
//        gyroturn(0.2, -90, 20);


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
