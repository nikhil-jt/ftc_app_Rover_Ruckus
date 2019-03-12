package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.BNO055IMU; import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator; import com.qualcomm.hardware.lynx.LynxModule; import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp; import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Func; import org.firstinspires.ftc.robotcore.external.navigation.Acceleration; import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit; import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder; import org.firstinspires.ftc.robotcore.external.navigation.AxesReference; import org.firstinspires.ftc.robotcore.external.navigation.Orientation; import org.firstinspires.ftc.robotcore.external.navigation.Position; import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import java.util.Locale;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@TeleOp(name="TrigTeleOp", group="Iterative Opmode")
@Disabled
public class TrigTeleOp extends OpMode
{
    private DcMotor fr ;
    private DcMotor fl ;
    private DcMotor br ;
    private DcMotor bl ;
    private BNO055IMU imu;
    private Orientation angles;
    private Acceleration gravity;
    private float lastAngle = 0;
    private float turning = 0;
    @Override
    public void init() {
        telemetry.addData("Status", "Initializing");
        fr = hardwareMap.dcMotor.get("fr");
        fl = hardwareMap.dcMotor.get ("fl") ;
        br = hardwareMap.dcMotor.get("br");
        bl = hardwareMap.dcMotor.get ("bl") ;
        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.REVERSE);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        gravity  = imu.getGravity();
        telemetry.addData("angle: ", angles.firstAngle);
        telemetry.addData("turning: ", turning);
        telemetry.addData("lastAngle: ", lastAngle);

        float x = gamepad1.right_stick_x;
        float y = gamepad1.right_stick_y;
        float z = gamepad1.left_stick_x;
        telemetry.addData("x: ", x);
        telemetry.addData("y: ", y);
        telemetry.addData("z: ", z);
        double power = Math.hypot(x, y);
        double angle = Math.atan2(y, x) - Math.PI/4;
//        if(Math.abs(angles.firstAngle - lastAngle) > 3 && z == 0) {
//            float offset = lastAngle - angles.firstAngle;
//            z -= offset/40;
//        } else if(x == 0 && y == 0 && z == 0 && turning == angles.firstAngle) {
//            lastAngle = angles.firstAngle;
//        }
//        turning = angles.firstAngle;
        double v1 = power * Math.sin(angle) + z;
        double v2 = power * Math.cos(angle) - z;
        double v3 = power * Math.cos(angle) + z;
        double v4 = power * Math.sin(angle) - z;
        if(gamepad1.dpad_up) {
            v1 = -0.6;
            v2 = -0.6;
            v3 = -0.6;
            v4 = -0.6;
        } else if(gamepad1.dpad_down) {
            v1 = 0.6;
            v2 = 0.6;
            v3 = 0.6;
            v4 = 0.6;
        } else if(gamepad1.dpad_left) {
            v1 = 0.4;
            v2 = -0.4;
            v3 = -0.4;
            v4 = 0.4;
        }  else if(gamepad1.dpad_right) {
            v1 = -0.4;
            v2 = 0.4;
            v3 = 0.4;
            v4 = -0.4;
        }
        fl.setPower(v1);
        fr.setPower(v2);
        bl.setPower(v3);
        br.setPower(v4);
    }
}