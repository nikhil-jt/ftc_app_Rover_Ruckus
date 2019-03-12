package org.firstinspires.ftc.teamcode;

import android.support.annotation.NonNull;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

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

import static java.lang.Math.abs;

public abstract class BaseAutonomous extends LinearOpMode {
    protected DcMotor left = null; //Inits the object to null
    protected DcMotor right = null; //Inits the object to null
    protected DcMotor belt = null; //Inits the object to null
    protected Servo leftservo = null; //Inits the object to null
    protected Servo rightservo = null; //Inits the object to null
    protected Servo jewelservo = null; //Inits the object to null
    protected ColorSensor colorSensor = null; //Inits the object to null
    protected BNO055IMU gyro = null;
    private ElapsedTime runtime = new ElapsedTime();
    static final double COUNTS_PER_MOTOR_REV = 4.0;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 72.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 3.54331;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double P_DRIVE_COEFF           = 0.10;     // Larger is more responsive, but also less stable

    @NonNull
    protected VuforiaTrackable initAndWaitForStart() {
        telemetry.addData("Wait", "NOT-READY");
        telemetry.update();

        left = hardwareMap.dcMotor.get("left_drive"); //Assign values to motors, servos, and the color sensor
        right = hardwareMap.dcMotor.get("right_drive");
        belt = hardwareMap.dcMotor.get("belt_drive");
        leftservo = hardwareMap.servo.get("left");
        rightservo = hardwareMap.servo.get("right");
        jewelservo = hardwareMap.servo.get("jewel");
        colorSensor = hardwareMap.colorSensor.get("color");
        right.setDirection(DcMotorSimple.Direction.REVERSE); //Reverses one motor or else they would be spinning opposite directions
        leftservo.setPosition(0.05); //Open left servo
        rightservo.setPosition(.9); //Open right servo
        jewelservo.setPosition(0.3); //Raises the servo slightly

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AX1o8pf/////AAAAGY0xa78pAEw0o9rshDeXgkh6k4SJzJzxfORsE36nn2lUWTyKP+ZPDlmLuVSdzn3Ca2+eTVxM9BBMdMeGdBXgnKxFFTzy7Cm5gSe53HAbid4vl1mngkdLislmQ/l/EBTizMsu4mRjybiA6uqcnFTmRZlBtGv1VvEjDwoOekbyxFFv6kDwZKsUJHabrlj0jSRFR5nFwwY3judR8TdjO9829Ny+JMT8n6S9+NQO/EYO0oZIFchzFo6cpVUW0aDEK/j/289v2ZuCA+vQnlVlqjd18g3R/zk/a+UTlgtSJg5lWfN8HZXdF7MYDJo4v22Er/Qe83DHnMkZBvcLm1w+4QNX8/tQcfYAiNMcG2aRLTg3XqMH";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        VuforiaLocalizer vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        VuforiaTrackables relicTrackables = vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTrackables.activate();
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate");
//        sleep(500);
//        vuforiaTurn((VuforiaTrackableDefaultListener) relicTemplate.getListener(), 0);

        gyro = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters gparameters = new BNO055IMU.Parameters();
        gparameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        gyro.initialize(gparameters);

        telemetry.addData("Init", "Ready");
        telemetry.update();

        waitForStart();
        return relicTemplate;
    }

    protected void vuforiaMove(VuforiaTrackableDefaultListener listener, double inchesToImage, int timeoutS) {
        double yDistance = 4.9;
        OpenGLMatrix pose = listener.getPose();
        if (pose != null) {
            VectorF translation = pose.getTranslation();
            yDistance = (translation.get(1)/25.4);
            telemetry.addData("Translation", translation);
        }
        telemetry.addData("Moving", yDistance);
        telemetry.update();
        encoderMove(0.3, yDistance - inchesToImage, timeoutS);
    }

    private void vuforiaTurn(VuforiaTrackableDefaultListener listener, double targetAngle) {
        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        while (!isStopRequested()) {
            OpenGLMatrix pose = listener.getPose();
            if(pose == null) {
                break;
            }
            Orientation orientation = Orientation.getOrientation(pose, AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
            double currangle2 = orientation.firstAngle;
            double angleDiff2 = -(targetAngle - currangle2);
            telemetry.update();
            if (angleDiff2 > 180) {
                angleDiff2 -= 360;
            }
            double power = (angleDiff2 / 180.0);
            double gyroTurnMinSpeed = 0.01;
            power = Range.clip(power, power < 0 ? -1 : gyroTurnMinSpeed, power < 0 ? -gyroTurnMinSpeed : 1);
            telemetry.addData("power", power);
            telemetry.addData("currangle2", currangle2);
            telemetry.update();
            if (abs(angleDiff2) < 0.5) {
                break;
            }
            right.setPower(power);
            left.setPower(-power);
        }
        right.setPower(0);
        left.setPower(0);
    }

    protected void encoderMove(double power, double inches, int timeoutS) {
        encoderMove(power, inches, inches, timeoutS);
    }

    protected void encoderMove(double power, double leftInches, double rightInches, int timeoutS) {
        if (!isStopRequested()) {
            int newLeftTarget = left.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            int newRightTarget = right.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            left.setTargetPosition(newLeftTarget);
            right.setTargetPosition(newRightTarget);
            left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            runtime.reset();
            left.setPower(Math.abs(power));
            right.setPower(Math.abs(power));
            while (!isStopRequested() && (runtime.seconds() < timeoutS) && (left.isBusy() || right.isBusy())) {
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d", left.getCurrentPosition(), right.getCurrentPosition());
                telemetry.update();
            }
            left.setPower(0);
            right.setPower(0);
            left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    @Deprecated
    public void move(double power, int time) {
        left.setPower(power);
        right.setPower(power);
        sleep(time);
        left.setPower(0);
        right.setPower(0);
    }

    @Deprecated
    public void turn(double leftpower, int time) {
        left.setPower(leftpower);
        right.setPower(-leftpower);
        sleep(time);
        left.setPower(0);
        right.setPower(0);
    }

    protected void gyroturn(double power, double angle, int timeoutS) {
        double currangle = gyro.getAngularOrientation().firstAngle;
        double angleDiff = angle - currangle;
        if (angleDiff > 180) {
            angleDiff -= 360;
        }

        double rightInches = (12.7 * 3.1415) * angleDiff / 360;
        encoderMove(power, -rightInches, rightInches, timeoutS);
        gyroturn(angle);
    }

    protected void gyroturn(double angle) {
        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        while (!isStopRequested()) {
            double currangle = gyro.getAngularOrientation().firstAngle;
            double angleDiff = angle - currangle;
            if (angleDiff > 180) {
                angleDiff -= 360;
            }
            double power = (angleDiff / 180.0);
            double gyroTurnMinSpeed = 0.15;
            power = Range.clip(power, power < 0 ? -1 : gyroTurnMinSpeed, power < 0 ? -gyroTurnMinSpeed : 1);
            telemetry.addData("power", power);
            telemetry.addData("currangle", currangle);
            telemetry.update();
            if (abs(angleDiff) < 0.5) {
                break;
            }
            right.setPower(power);
            left.setPower(-power);
        }
        right.setPower(0);
        left.setPower(0);
    }

    public int gyroturn(double power, int angle) {
        double currangle = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        double diffangle = abs(currangle - angle);
        if (currangle > angle) {
            right.setPower(-power);
            left.setPower(power);
            while (gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle > angle + 1) {
                double currpro = abs(gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle - currangle);
                if (currpro < diffangle / 2) {
                    telemetry.addData("pos", 1);
                    double newpower = power * (1 + (currpro / (diffangle / 2)));
                    right.setPower(-newpower);
                    left.setPower(newpower);
                } else {
                    telemetry.addData("pos", 2);
                    double newpower = power * (1 + ((diffangle - currpro) / (diffangle / 2)));
                    right.setPower(-newpower);
                    left.setPower(newpower);
                }
                telemetry.addData("right", right.getPower());
                telemetry.addData("left", left.getPower());
                telemetry.update();
                sleep(10);
            }
            right.setPower(0);
            left.setPower(0);
        } else {
            right.setPower(power);
            left.setPower(-power);
            while (gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle < angle - 1) {
                double currpro = abs(gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle - currangle);
                if (currpro < diffangle / 2) {
                    telemetry.addData("pos", 1);
                    double newpower = power * (1 + (currpro / (diffangle / 2)));
                    right.setPower(newpower);
                    left.setPower(-newpower);
                } else {
                    telemetry.addData("pos", 2);
                    double newpower = power * (1 + ((diffangle - currpro) / (diffangle / 2)));
                    right.setPower(newpower);
                    left.setPower(-newpower);
                }
                telemetry.addData("right", right.getPower());
                telemetry.addData("left", left.getPower());
                telemetry.update();
                sleep(10);
            }
            right.setPower(0);
            left.setPower(0);
        }
        telemetry.addData("angle", gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
        telemetry.update();
        return 1;
    }

    protected void jewelMove(boolean redSide, VuforiaTrackable relicTemplate) {
        leftservo.setPosition(0.45);
        rightservo.setPosition(0.5);
        sleep(500);
        belt.setPower(1);
        sleep(1000);
        belt.setPower(0);

//        TODO adjust 2.0 - since it is a guess
//        vuforiaMove((VuforiaTrackableDefaultListener) relicTemplate.getListener(), 2.35, 20);
        encoderMove(0.5, 2.1, 60);

        jewelservo.setPosition(0.88);
        sleep(700);
        jewelservo.setPosition(1.0);
        sleep(200);

        int red = colorSensor.red();
        int blue = colorSensor.blue();
        telemetry.addData("Red", red);
        telemetry.addData("Blue", blue);
        telemetry.update();
        sleep(5000);

        if (red < blue) {
            telemetry.addData("Color", "Blue");
            telemetry.update();
//            sleep(1000);

            double rightLength = redSide ? -2 : 2;
            encoderMove(0.5, -rightLength, rightLength, 40);
            jewelservo.setPosition(0.3);
            sleep(500);
        } else {
            telemetry.addData("Color", "Red");
            telemetry.update();
//            sleep(1000);

            double rightLength = redSide ? 2 : -2;
            encoderMove(0.5, -rightLength, rightLength, 40);
            jewelservo.setPosition(0.3);
            sleep(500);
        }
    }

    static final double     P_TURN_COEFF            = 0.1;     // Larger is more responsive, but also less stable
    /**
     *  Method to spin on central axis to point in a new direction.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the heading (angle)
     *  2) Driver stops the opmode running.
     *
     * @param speed Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     */
    public void gyroTurn (  double speed, double angle) {
        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // keep looping while we are still active, and not on heading.
        while (!isStopRequested() && !onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
        }
    }

    /**
     *  Method to obtain & hold a heading for a finite amount of time
     *  Move will stop once the requested time has elapsed
     *
     * @param speed      Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     * @param holdTime   Length of time (in seconds) to hold the specified heading.
     */
    public void gyroHold( double speed, double angle, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (!isStopRequested() && (holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angle, P_TURN_COEFF);
            telemetry.update();
        }

        // Stop all motion;
        left.setPower(0);
        right.setPower(0);
    }

    static final double     HEADING_THRESHOLD       = 0.1 ;      // As tight as we can make it with an integer gyro
    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed     Desired speed of turn.
     * @param angle     Absolute Angle (in Degrees) relative to last gyro reset.
     *                  0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                  If a relative angle is required, add/subtract from current heading.
     * @param PCoeff    Proportional Gain coefficient
     * @return
     */
    boolean onHeading(double speed, double angle, double PCoeff) {
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        double error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            rightSpeed  = speed * steer;
            leftSpeed   = -rightSpeed;
        }

        // Send desired speeds to motors.
        left.setPower(leftSpeed);
        right.setPower(rightSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }

    public void gyroDrive(double speed, double distance, double angle, Boolean turn) {
        if(turn) {
            gyroturn(0.3, angle, 20);
        }
        int     newLeftTarget;
        int     newRightTarget;
        int     moveCounts;
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;
        moveCounts = (int)(distance * COUNTS_PER_INCH);
        newLeftTarget = left.getCurrentPosition() + moveCounts;
        newRightTarget = right.getCurrentPosition() + moveCounts;
        left.setTargetPosition(newLeftTarget);
        right.setTargetPosition(newRightTarget);
        left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        speed = Range.clip(Math.abs(speed), 0.0, 1.0);
        left.setPower(speed);
        right.setPower(speed);
        while (opModeIsActive() && (left.isBusy() && right.isBusy())) {
            error = getError(angle);
            steer = getSteer(error, P_DRIVE_COEFF);
            if (distance < 0)
                steer *= -1.0;
            leftSpeed = speed - steer;
            rightSpeed = speed + steer;
            max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
            if (max > 1.0)
            {
                leftSpeed /= max;
                rightSpeed /= max;
            }
            left.setPower(leftSpeed);
            right.setPower(rightSpeed);
            telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
            telemetry.addData("Target",  "%7d:%7d",      newLeftTarget,  newRightTarget);
            telemetry.addData("Actual",  "%7d:%7d",      left.getCurrentPosition(),
                    right.getCurrentPosition());
            telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
            telemetry.update();
        }
        left.setPower(0);
        right.setPower(0);
        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    /**
     * getError determines the error between the target angle and the robot's current heading
     * @param   targetAngle  Desired angle (relative to global reference established at last Gyro Reset).
     * @return  error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     *          +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }
}
