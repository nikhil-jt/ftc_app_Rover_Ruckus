package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.support.annotation.NonNull;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

import static java.lang.Math.abs;

public abstract class BaseAutonomous9511 extends LinearOpMode {
    static {
        System.loadLibrary("opencv_java3");
    }
    protected Servo intake;
    protected Servo box;
    protected Servo intakebox;
    protected DcMotor br ;
    protected DcMotorEx scoringarm;
    protected DcMotor bl ;
    protected DcMotor linear;
    protected DcMotor up;
    protected double intakePosition;
    protected BNO055IMU gyro = null;
    protected ElapsedTime runtime = new ElapsedTime();
    static final double COUNTS_PER_MOTOR_REV = 28;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 40;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 3.5;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double HEADING_THRESHOLD       = 0.5 ;      // As tight as we can make it with our gyro
    static final double P_DRIVE_COEFF           = 0.025;     // Larger is more responsive, but also less stable
    static final double COUNTS_PER_MOTOR_REV2 = 4;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION2 = 72.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES2 = 3.54331;     // For figuring circumference
    static final double COUNTS_PER_INCH2 = (COUNTS_PER_MOTOR_REV2 * DRIVE_GEAR_REDUCTION2) / (WHEEL_DIAMETER_INCHES2 * 3.1415);
    static final double wheeldist = 18;

    @NonNull
    protected VuforiaLocalizer initAndWaitForStart() {
        VuforiaLocalizer vuforiaLocalizer = initRobot();
        waitForStart();
        return vuforiaLocalizer;
    }

    @NonNull
    protected VuforiaLocalizer initRobot() {
        telemetry.addData("Wait", "NOT-READY");
        telemetry.update();
        br = hardwareMap.dcMotor.get("br");
        bl = hardwareMap.dcMotor.get("bl");
        up = hardwareMap.dcMotor.get("up");
        up.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        up.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        up.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        scoringarm = (DcMotorEx) hardwareMap.dcMotor.get("scoring");
        scoringarm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        scoringarm.setPower(0);
        scoringarm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intake = hardwareMap.servo.get("intake");
        intakebox = hardwareMap.servo.get("intakebox");
        box = hardwareMap.servo.get("box");
        intakePosition = 0;
        intake.setPosition(intakePosition);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
//        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AUExoFn/////AAABmWvBwndvLEMtq9xk1w1qb9BS677KKMtCwZv+EA+7xcPlQYrqQppMynhm/gAPyylD5gavqefApxBQysR8dh3TnfPZshi8nCg4JGPkjY7y0D0Bbvt3Mgqw0Gokq2OMmrjSvrUJqJIHhHD2soo+Rstw0E4+QX2oH/pnVPfH6KIRpALZpVB3pkzF/MB5xtmGSnNPFiNWYFP8rESUuzlQ/k5JJz/XZ85mWqTNJUgb7Ne4IFJxWJGbFyw6379bCD2ixFw7z+vcN3mHlZa2XW6h1fhwAKSN0kROCmtqPwK8eKtByk1IRzzQN0bKooC8NmeCfAMzJrnRdYKp3FzIkaAvwnYSxslVV/TIOpXvbCHGHo2B2LA1";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        VuforiaLocalizer vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);
        vuforia.setFrameQueueCapacity(5);
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
        return vuforia;
    }

    protected void setLinear(double power, int count) {
        linear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linear.setTargetPosition(count);
        linear.setPower(power);
        while(opModeIsActive() && !linear.isBusy()) {
            telemetry.addData("linear", linear.getCurrentPosition());
        }
        linear.setPower(0);
    }


//    protected void vuforiaMove(VuforiaTrackableDefaultListener listener, double inchesToImage, int timeoutS) {
//        double yDistance = 4.9;
//        OpenGLMatrix pose = listener.getPose();
//        if (pose != null) {
//            VectorF translation = pose.getTranslation();
//            yDistance = (translation.get(1)/25.4);
//            telemetry.addData("Translation", translation);
//        }
//        telemetry.addData("Moving", yDistance);
//        telemetry.update();
//        encoderMove(0.3, yDistance - inchesToImage, timeoutS);
//    }

//    protected void vuforiaTurn(VuforiaTrackableDefaultListener listener, double targetAngle) {
//        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        while (!isStopRequested()) {
//            OpenGLMatrix pose = listener.getPose();
//            if(pose == null) {
//                break;
//            }
//            Orientation orientation = Orientation.getOrientation(pose, AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
//            double currangle2 = orientation.firstAngle;
//            double angleDiff2 = -(targetAngle - currangle2);
//            telemetry.update();
//            if (angleDiff2 > 180) {
//                angleDiff2 -= 360;
//            }
//            double power = (angleDiff2 / 180.0);
//            double gyroTurnMinSpeed = 0.01;
//            power = Range.clip(power, power < 0 ? -1 : gyroTurnMinSpeed, power < 0 ? -gyroTurnMinSpeed : 1);
//            telemetry.addData("power", power);
//            telemetry.addData("currangle2", currangle2);
//            telemetry.update();
//            if (abs(angleDiff2) < 0.5) {
//                break;
//            }
//            right.setPower(power);
//            left.setPower(-power);
//        }
//        right.setPower(0);
//        left.setPower(0);
//    }

    protected void gyroTurn(double speed, double angleDegrees) {
        angleDegrees = -angleDegrees;
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if(abs(getError(angleDegrees)) < 8) {
            telemetry.addData("Error:", abs(getError(angleDegrees)));
            telemetry.addData("Angle:", angleDegrees);
            telemetry.update();
//            sleep(1000);
            return;
        }
        while (opModeIsActive()) {
            double error = getError(angleDegrees);

            double steer = getSteer(error, P_DRIVE_COEFF);
            double rightSpeed  = speed * steer;
            if (rightSpeed < 0 && rightSpeed > -0.01) {
                rightSpeed = -0.01;
            }
            if (rightSpeed > 0 && rightSpeed < 0.01) {
                rightSpeed = 0.01;
            }
            double leftSpeed = -rightSpeed;
            float angle = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            telemetry.addData("Angle/Err/St",  "%5.1f/%5.1f/%5.1f",  angle, error, steer);
            telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
            telemetry.update();

            if (Math.abs(error) <= HEADING_THRESHOLD) {
                break;
            }

            bl.setPower(leftSpeed);
            br.setPower(rightSpeed);
//            fl.setPower(leftSpeed);
//            fr.setPower(rightSpeed);
        }
        bl.setPower(0);
        br.setPower(0);
//        fl.setPower(0);
//        fr.setPower(0);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    protected void encoderTurn(double power, double angle, int timeoutS) {
        double leftInches = (angle/360)*wheeldist*3.1415;
        telemetry.addData("turnonches", leftInches);
        telemetry.update();
        sleep(2000);
        encoderMove(power, leftInches, -leftInches, timeoutS);
//        double rightInches = -leftInches;
//        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        int newLeftTarget = bl.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
//        int newRightTarget = br.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
//        runtime.reset();
//        bl.setPower(leftInches < 0 ? -abs(power) : abs(power));
//        br.setPower(rightInches < 0 ? -abs(power) : abs(power));
//        while (!isStopRequested() && (runtime.seconds() < timeoutS)) {
//            telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
//            telemetry.addData("Path2", "Running at %7d :%7d", bl.getCurrentPosition(), br.getCurrentPosition());
//            telemetry.update();
//            if ((leftInches < 0 ? (bl.getCurrentPosition() - newLeftTarget < 0) : (bl.getCurrentPosition() - newLeftTarget > 0)) || (rightInches < 0 ? (br.getCurrentPosition() - newRightTarget < 0) : (br.getCurrentPosition() - newRightTarget > 0))) {
//                break;
//            }
//        }
//        bl.setPower(0);
//        br.setPower(0);
    }
    protected void encoderMove(double power, double inches, int timeoutS) {
        encoderMove(power, inches, inches, timeoutS);
    }

    protected void encoderMove(double power, double leftInches, double rightInches, int timeoutS) {
        turn2(power, leftInches, rightInches, timeoutS);
//        encoderMove1(power, leftInches, rightInches, timeoutS);
//        if (!isStopRequested() && !(leftInches == 0 && rightInches == 0)) {
//            int newLeftTarget = bl.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
//            int newfLeftTarget = fl.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH2);
//            int newfRightTarget = fr.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH2);
//            int newRightTarget = br.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
//            bl.setTargetPosition(newLeftTarget);
//            fl.setTargetPosition(newfLeftTarget);
//            fr.setTargetPosition(newfRightTarget);
//            br.setTargetPosition(newRightTarget);
//            bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            br.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            runtime.reset();
//            bl.setPower(Math.abs(power));
//            fl.setPower(Math.abs(power));
//            br.setPower(Math.abs(power));
//            fr.setPower(Math.abs(power));
//            while (!isStopRequested() && (runtime.seconds() < timeoutS) && (bl.isBusy() || br.isBusy() || fl.isBusy() || fr.isBusy())) {
//                telemetry.addData("Path1", "Running to %7d :%7d :%7d :%7d", newLeftTarget, newRightTarget, newfLeftTarget, newfRightTarget);
//                telemetry.addData("Path2", "Running at %7d :%7d :%7d :%7d", bl.getCurrentPosition(), br.getCurrentPosition(), fl.getCurrentPosition(), fr.getCurrentPosition());
//                telemetry.update();
//            }
//            bl.setPower(0);
//            fl.setPower(0);
//            br.setPower(0);
//            fr.setPower(0);
//            bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        }
    }

    protected void encoderMove1(double maxPower, double leftInches, double rightInches, int timeoutS) {
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        int newLeftTarget = bl.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
        int newRightTarget = br.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
        runtime.reset();
        bl.setPower(leftInches < 0 ? -abs(maxPower) : abs(maxPower));
        br.setPower(rightInches < 0 ? -abs(maxPower) : abs(maxPower));
//        fl.setPower(leftInches < 0 ? -abs(maxPower) : abs(maxPower));
//        fr.setPower(rightInches < 0 ? -abs(maxPower) : abs(maxPower));
        while (!isStopRequested() && (runtime.seconds() < timeoutS)) {
            telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
            telemetry.addData("Path2", "Running at %7d :%7d", bl.getCurrentPosition(), br.getCurrentPosition());
            telemetry.update();
            if ((leftInches < 0 ? (bl.getCurrentPosition() - newLeftTarget < 0) : (bl.getCurrentPosition() - newLeftTarget > 0)) || (rightInches < 0 ? (br.getCurrentPosition() - newRightTarget < 0) : (br.getCurrentPosition() - newRightTarget > 0))) {
                break;
            }
        }
        bl.setPower(0);
        br.setPower(0);
//        fl.setPower(0);
//        fr.setPower(0);
    }

    public void turn2(double maxPower, double leftInches, double rightInches, int timeoutS) {
        int newLeftTarget = bl.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
        int newRightTarget = br.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
        bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        br.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        int oldBackLeft = bl.getCurrentPosition();
        int oldBackRight = br.getCurrentPosition();
        bl.setTargetPosition(newLeftTarget);
        br.setTargetPosition(newRightTarget);
        runtime.reset();
        bl.setPower(maxPower);
        br.setPower(maxPower);
        while (opModeIsActive() &&
                (runtime.seconds() < timeoutS) &&
                (bl.isBusy() || br.isBusy())) {
            telemetry.addData("Path1 - to", "Running from %3d :%3d to %3d :%3d", oldBackLeft, oldBackRight, newLeftTarget, newRightTarget);
            telemetry.addData("Path1 - current", "Running at %3d :%3d", bl.getCurrentPosition(), br.getCurrentPosition());
            telemetry.update();
        }
        bl.setPower(0);
        br.setPower(0);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    protected Rect locateGold(Mat workingMat) {
        if (workingMat != null) { //does workingmat exist
            workingMat = new Mat(workingMat, new Rect(0, 0, workingMat.width(), workingMat.height())); // trim workingmat to only half the screen, reducing the amount of pixels the function has to check
            Imgproc.GaussianBlur(workingMat, workingMat, new Size(5, 5), 0); //blurs it, which basically allows for smoother edges and better detection
            Mat maskYellow = yellowFilter(workingMat); //get the yellow layer from RGB values
            workingMat.release(); //release workingmat from memory to save processing power

            List<MatOfPoint> contoursYellow = new ArrayList<>(); //initializes array for contours
            Mat hierarchy = new Mat(); //creates new mat
            Imgproc.findContours(maskYellow, contoursYellow, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE); //finds all sets of closed edges in the picture
            maskYellow.release(); //more release
            workingMat.release(); //more release
            return getBestSquareWithMaxArea(contoursYellow);
        }
        return null;
    }

    protected Rect getBestSquareWithMaxArea(List<MatOfPoint> contoursYellow) {
        Rect bestRect = null;
        float bestDifference = Float.MAX_VALUE; //intialize best score to max so next contour will win
        for (MatOfPoint matOfPoint : contoursYellow) { //for each contour set
//            telemetry.addData("mat", matOfPoint);
            Rect rect = Imgproc.boundingRect(matOfPoint); //gets rectangle that encompasses it
            double cubeRatio = Math.max(Math.abs(rect.height/rect.width), Math.abs(rect.width/rect.height)); //finds the squariness of the rectangle by doing width/height or height/width, closer to 1 is more square
            float ratioDifference = (float) Math.abs(cubeRatio - 1.0); //subtracts 1 so that best is closer to 0
            float contourArea = (float) Imgproc.contourArea(matOfPoint); //finds area of rectangle
//            float score = (float) (ratioDifference + (-contourArea * 0.001));
//            float score = (float) (ratioDifference + (Math.abs(3600 - contourArea) * 0.05)); //calculates the score by weighting area and ratio, lower score is better
            float score = (float) (ratioDifference + (Math.abs(3600 - contourArea) * 0.05)); //calculates the score by weighting area and ratio, lower score is better
            //            float score = ratioDifference;
            if (score < bestDifference) { //checks if score is better, then replaces best score
                bestDifference = score;
                bestRect = rect;
            }
        }
        return bestRect;
    }

    protected Mat yellowFilter(Mat input) {
        double threshold = 70;
        //threshold of yellow
        Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2YUV); //gets the color channel
        Imgproc.GaussianBlur(input,input,new Size(3,3),0); //another blur
        List<Mat> channels = new ArrayList<>(); //gets color channels
        Core.split(input, channels); //splits color channels
        Mat mask = new Mat();
        if (channels.size() > 0){
            Imgproc.threshold(channels.get(1), mask, threshold, 255, Imgproc.THRESH_BINARY_INV); //thresholds the mask
        }
        return mask;
    }

    protected Mat getMatFromVuforia(VuforiaLocalizer vuforia) {
        VuforiaLocalizer.CloseableFrame frame = null; //gets frame
        try {
            frame = vuforia.getFrameQueue().take(); //takes frame
            for (int i = 0; i < frame.getNumImages(); i++) { //goes through frames
                if (frame.getImage(i).getFormat() == PIXEL_FORMAT.RGB565) { //checks if frame is right color config
                    Image rgb = frame.getImage(i);
                    Bitmap bm = Bitmap.createBitmap(rgb.getWidth(), rgb.getHeight(), Bitmap.Config.RGB_565);
                    bm.copyPixelsFromBuffer(rgb.getPixels());
                    Mat mat = new Mat(bm.getWidth(), bm.getHeight(), CvType.CV_8UC1);
                    Utils.bitmapToMat(bm, mat);
                    bm.recycle();
                    return mat; //all this is to convert it from a vuforia mat to bitmap to mat
                }
            }
        } catch (Exception e) {
            e.printStackTrace();
        } finally {
            if (frame != null) {
                frame.close();
            }
        }
        return null;
    }
    protected int getCameraWidth(VuforiaLocalizer vuforia) {
        try {
            VuforiaLocalizer.CloseableFrame frame = vuforia.getFrameQueue().peek();
            for (int i = 0; i < frame.getNumImages(); i++) {
                if (frame.getImage(i).getFormat() == PIXEL_FORMAT.RGB565) {
                    Image rgb = frame.getImage(i);
                    return rgb.getWidth(); //gets width
                }
            }
        } catch (Exception e) {
            e.printStackTrace();
        }
        return 0;
    }

//    protected void gyroturn(double power, double angle, int timeoutS) {
//        double currangle = gyro.getAngularOrientation().firstAngle;
//        double angleDiff = angle - currangle;
//        if (angleDiff > 180) {
//            angleDiff -= 360;
//        }
//
//        double rightInches = (12.7 * 3.1415) * angleDiff / 360;
//        encoderMove(power, -rightInches, rightInches, timeoutS);
//        gyroturn(angle);
//    }

//    protected void gyroturn(double angle) {
//        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        while (!isStopRequested()) {
//            double currangle = gyro.getAngularOrientation().firstAngle;
//            double angleDiff = angle - currangle;
//            if (angleDiff > 180) {
//                angleDiff -= 360;
//            }
//            double power = (angleDiff / 180.0);
//            double gyroTurnMinSpeed = 0.15;
//            power = Range.clip(power, power < 0 ? -1 : gyroTurnMinSpeed, power < 0 ? -gyroTurnMinSpeed : 1);
//            telemetry.addData("power", power);
//            telemetry.addData("currangle", currangle);
//            telemetry.update();
//            if (abs(angleDiff) < 0.5) {
//                break;
//            }
//            fr.setPower(power);
//            fl.setPower(-power);
//        }
//        fr.setPower(0);
//        fl.setPower(0);
//    }

//    public int gyroturn(double power, int angle) {
//        double currangle = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
//        double diffangle = abs(currangle - angle);
//        if (currangle > angle) {
//            right.setPower(-power);
//            left.setPower(power);
//            while (gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle > angle + 1) {
//                double currpro = abs(gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle - currangle);
//                if (currpro < diffangle / 2) {
//                    telemetry.addData("pos", 1);
//                    double newpower = power * (1 + (currpro / (diffangle / 2)));
//                    right.setPower(-newpower);
//                    left.setPower(newpower);
//                } else {
//                    telemetry.addData("pos", 2);
//                    double newpower = power * (1 + ((diffangle - currpro) / (diffangle / 2)));
//                    right.setPower(-newpower);
//                    left.setPower(newpower);
//                }
//                telemetry.addData("right", right.getPower());
//                telemetry.addData("left", left.getPower());
//                telemetry.update();
//                sleep(10);
//            }
//            right.setPower(0);
//            left.setPower(0);
//        } else {
//            right.setPower(power);
//            left.setPower(-power);
//            while (gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle < angle - 1) {
//                double currpro = abs(gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle - currangle);
//                if (currpro < diffangle / 2) {
//                    telemetry.addData("pos", 1);
//                    double newpower = power * (1 + (currpro / (diffangle / 2)));
//                    right.setPower(newpower);
//                    left.setPower(-newpower);
//                } else {
//                    telemetry.addData("pos", 2);
//                    double newpower = power * (1 + ((diffangle - currpro) / (diffangle / 2)));
//                    right.setPower(newpower);
//                    left.setPower(-newpower);
//                }
//                telemetry.addData("right", right.getPower());
//                telemetry.addData("left", left.getPower());
//                telemetry.update();
//                sleep(10);
//            }
//            right.setPower(0);
//            left.setPower(0);
//        }
//        telemetry.addData("angle", gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
//        telemetry.update();
//        return 1;
//    }

//    protected void jewelMove(boolean redSide, VuforiaTrackable relicTemplate) {
////        sleep(500);
////        belt.setPower(1);
////        sleep(1000);
////        belt.setPower(0);
//
////        TODO adjust 2.0 - since it is a guess
////        vuforiaMove((VuforiaTrackableDefaultListener) relicTemplate.getListener(), 2.35, 20);
////        encoderMove(0.5, 2.1, 60);
//
//        jewelservo.setPosition(0.88);
//        sleep(700);
//        jewelservo.setPosition(0.95);
//        sleep(200);
//
//        int red = colorSensor.red();
//        int blue = colorSensor.blue();
//        telemetry.addData("Red", red);
//        telemetry.addData("Blue", blue);
//        telemetry.update();
//        sleep(5000);
//
//        if (redSide) {
//            if (red < blue) {
//                telemetry.addData("Color", "Blue - redside");
//                telemetry.update();
////                move(-0.5, 400);
////                encoderMove(0.8, -10, -10, 40);
//            } else {
//                telemetry.addData("Color", "Red - redside");
//                telemetry.update();
////                move(0.5, 400);
////                encoderMove(0.8, 10, 10, 40);
//            }
//        } else {
//            if (red < blue) {
//                telemetry.addData("Color", "Blue - blueside");
//                telemetry.update();
////                move(0.5, 400);
////                encoderMove(0.8, 4, -4, 40);
//            } else {
//                telemetry.addData("Color", "Red - blueside");
//                telemetry.update();
////                move(-0.5, 400);
////                encoderMove(0.8, 4, 4, 40);
//            }
//        }
//        sleep(500);
//        jewelservo.setPosition(0.3);
//        sleep(500);
//    }

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
//    public void gyroTurn (  double speed, double angle) {
//        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        // keep looping while we are still active, and not on heading.
//        while (!isStopRequested() && !onHeading(speed, angle, P_TURN_COEFF)) {
//            // Update telemetry & Allow time for other processes to run.
//            telemetry.update();
//        }
//    }

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
//    public void gyroHold( double speed, double angle, double holdTime) {
//
//        ElapsedTime holdTimer = new ElapsedTime();
//
//        // keep looping while we have time remaining.
//        holdTimer.reset();
//        while (!isStopRequested() && (holdTimer.time() < holdTime)) {
//            // Update telemetry & Allow time for other processes to run.
//            onHeading(speed, angle, P_TURN_COEFF);
//            telemetry.update();
//        }
//
//        // Stop all motion;
//        left.setPower(0);
//        right.setPower(0);
//    }

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
//    boolean onHeading(double speed, double angle, double PCoeff) {
//        double   steer ;
//        boolean  onTarget = false ;
//        double leftSpeed;
//        double rightSpeed;
//
//        // determine turn power based on +/- error
//        double error = getError(angle);
//
//        if (Math.abs(error) <= HEADING_THRESHOLD) {
//            steer = 0.0;
//            leftSpeed  = 0.0;
//            rightSpeed = 0.0;
//            onTarget = true;
//        }
//        else {
//            steer = getSteer(error, PCoeff);
//            rightSpeed  = speed * steer;
//            leftSpeed   = -rightSpeed;
//        }
//
//        // Send desired speeds to motors.
//        left.setPower(leftSpeed);
//        right.setPower(rightSpeed);
//
//        // Display it for the driver.
//        telemetry.addData("Target", "%5.2f", angle);
//        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
//        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);
//
//        return onTarget;
//    }

//    public void gyroDrive(double speed, double distance, double angle, Boolean turn) {
//        if(turn) {
//            gyroturn(0.3, angle, 20);
//        }
//        int     newLeftTarget;
//        int     newRightTarget;
//        int     moveCounts;
//        double  max;
//        double  error;
//        double  steer;
//        double  leftSpeed;
//        double  rightSpeed;
//        moveCounts = (int)(distance * COUNTS_PER_INCH);
//        newLeftTarget = left.getCurrentPosition() + moveCounts;
//        newRightTarget = right.getCurrentPosition() + moveCounts;
//        left.setTargetPosition(newLeftTarget);
//        right.setTargetPosition(newRightTarget);
//        left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        speed = Range.clip(Math.abs(speed), 0.0, 1.0);
//        left.setPower(speed);
//        right.setPower(speed);
//        while (opModeIsActive() && (left.isBusy() && right.isBusy())) {
//            error = getError(angle);
//            steer = getSteer(error, P_DRIVE_COEFF);
//            if (distance < 0)
//                steer *= -1.0;
//            leftSpeed = speed - steer;
//            rightSpeed = speed + steer;
//            max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
//            if (max > 1.0)
//            {
//                leftSpeed /= max;
//                rightSpeed /= max;
//            }
//            left.setPower(leftSpeed);
//            right.setPower(rightSpeed);
//            telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
//            telemetry.addData("Target",  "%7d:%7d",      newLeftTarget,  newRightTarget);
//            telemetry.addData("Actual",  "%7d:%7d",      left.getCurrentPosition(),
//                    right.getCurrentPosition());
//            telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
//            telemetry.update();
//        }
//        left.setPower(0);
//        right.setPower(0);
//        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//    }
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
