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
//@Disabled
public class VuforiaTest extends BaseAutonomous {
    public void runOpMode() {
        VuforiaTrackable relicTemplate = initAndWaitForStart();
        while(opModeIsActive()) {
            OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) relicTemplate.getListener()).getPose();
            if (pose != null) {
                Orientation orientation = Orientation.getOrientation(pose, AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
                telemetry.addData("FirstAngle, y distance", "%.2f, %.2f", orientation.firstAngle, pose.getTranslation().get(2));
            } else {
                telemetry.addData("Detected: ", false);
            }
            telemetry.update();
        }
    }
}
