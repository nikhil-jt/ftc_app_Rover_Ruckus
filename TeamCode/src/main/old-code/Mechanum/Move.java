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

@Autonomous(name="Move", group="Linear Opmode")
//@Disabled
public class Move extends BaseAutonomous
{

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
        gyroDrive(0.5, 21.5, 0 + angleOffset, false);

        if (vuMark == RelicRecoveryVuMark.RIGHT) {
            gyroDrive(0.7, 8.5, -80 + angleOffset, true);
        } else if (vuMark == RelicRecoveryVuMark.CENTER) {
            gyroDrive(0.7, 8.5, -60 + angleOffset, true);
        } else {//LEFT or unable to see vuMark
            gyroDrive(0.7, 12.0, -40 + angleOffset, true);
        }

        belt.setPower(-1);
        sleep(950);
        leftservo.setPosition(0.1); //Open left servo
        rightservo.setPosition(0.9); //Open right servo
        belt.setPower(0);
        gyroturn(0.3, -90 + angleOffset, 20);
        encoderMove(0.6, -5, 1);
        encoderMove(0.6, 7, 1);
        encoderMove(0.6, -5, 1);
        telemetry.addData("currangle", gyro.getAngularOrientation().firstAngle);
        telemetry.update();
        sleep(500);
    }
}