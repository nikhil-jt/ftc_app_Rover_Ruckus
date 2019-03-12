package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

//import static com.sun.tools.javac.util.Constants.format;

/**
 * Created by jitto on 06/10/17.
 */
//
@Autonomous(name="test", group="Linear Opmode")
@Disabled
public class VuforiaTest extends LinearOpMode{
    DcMotor left = null;
    DcMotor right = null;
    DcMotor belt = null;
    Servo leftservo = null;
    Servo rightservo = null;
    Servo jewelservo = null;
    ColorSensor colorSensor = null;


    public void runOpMode() {
        try {
            VuforiaLocalizer vuforia;
            telemetry.addData("1", true);
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            telemetry.addData("2", true);
            VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
            telemetry.addData("3", true);
            parameters.vuforiaLicenseKey = "AX1o8pf/////AAAAGY0xa78pAEw0o9rshDeXgkh6k4SJzJzxfORsE36nn2lUWTyKP+ZPDlmLuVSdzn3Ca2+eTVxM9BBMdMeGdBXgnKxFFTzy7Cm5gSe53HAbid4vl1mngkdLislmQ/l/EBTizMsu4mRjybiA6uqcnFTmRZlBtGv1VvEjDwoOekbyxFFv6kDwZKsUJHabrlj0jSRFR5nFwwY3judR8TdjO9829Ny+JMT8n6S9+NQO/EYO0oZIFchzFo6cpVUW0aDEK/j/289v2ZuCA+vQnlVlqjd18g3R/zk/a+UTlgtSJg5lWfN8HZXdF7MYDJo4v22Er/Qe83DHnMkZBvcLm1w+4QNX8/tQcfYAiNMcG2aRLTg3XqMH";
            telemetry.addData("4", true);
            parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
            telemetry.addData("5", true);
            vuforia = ClassFactory.createVuforiaLocalizer(parameters);
            telemetry.addData("6", true);
            VuforiaTrackables relicTrackables = vuforia.loadTrackablesFromAsset("RelicVuMark");
            telemetry.addData("7", true);
            VuforiaTrackable relicTemplate = relicTrackables.get(0);
            telemetry.addData("8", true);
            relicTemplate.setName("relicVuMarkTemplate");
            telemetry.addData("9", true);
        } catch (Exception e) {
            telemetry.update();
        }


        waitForStart();
//        while(opModeIsActive()) {
//            OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) relicTemplate.getListener()).getPose();
////            telemetry.addData("Pose", format(pose));
//
///* We further illustrate how to decompose the pose into useful rotational and
//* translational components */
//            if (pose != null) {
//                telemetry.addData("Detected: ", true);
//
//                VectorF trans = pose.getTranslation();
//                Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
//
//                // Extract the X, Y, and Z components of the offset of the target relative to the robot
//                double tX = trans.get(0);
//                double tY = trans.get(1);
//                double tZ = trans.get(2);
//
//                // Extract the rotational components of the target relative to the robot
//                double rX = rot.firstAngle;
//                double rY = rot.secondAngle;
//                double rZ = rot.thirdAngle;
//            } else {
//                telemetry.addData("Detected: ", false);
//            }
//            telemetry.update();
//        }
    }
}
