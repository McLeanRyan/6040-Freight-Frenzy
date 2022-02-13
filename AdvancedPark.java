package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.ZYX;

@Autonomous
public class AdvancedPark extends LinearOpMode {
    DcMotor leftFront, leftBack, rightFront, rightBack;
    DcMotor lift, intake;
    CRServo box, duck;

    double ticksPerMotorRev = 288;
    double driveGearReduction = 1;
    double wheelDiameterInches = 3.77953;
    double ticksPerInch = (ticksPerMotorRev * driveGearReduction) / (wheelDiameterInches * 3.14159265359);

    boolean far = false;
    double distance = 3;
    int delay = 0;
    boolean left = true;

    boolean a = false;
    int Where =0;
    BNO055IMU imu;
    private Orientation angles;

    double kP = 0.005;
    double kD = 0.01;
    double kI = 0.00008;

    double totalError = 0;
    double lastAngle = 0;

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Mode", "Initialized");
        telemetry.update();

        rightFront = hardwareMap.dcMotor.get("RF");
        rightBack = hardwareMap.dcMotor.get("RB");
        leftFront = hardwareMap.dcMotor.get("LF");
        leftBack = hardwareMap.dcMotor.get("LB");
        intake = hardwareMap.dcMotor.get("intake");
        lift = hardwareMap.dcMotor.get("lift");

        duck = hardwareMap.crservo.get("duck");
        box = hardwareMap.crservo.get("box");

        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
        intake.setPower(0);
        lift.setPower(0);

        box.setPower(0);
        duck.setPower(0);

        telemetry.addData("leftFront Starting Pos", leftFront.getCurrentPosition());
        telemetry.addData("leftBack Starting Pos", leftBack.getCurrentPosition());
        telemetry.addData("rightFront Starting Pos", rightFront.getCurrentPosition());
        telemetry.addData("rightBack Starting Pos", rightBack.getCurrentPosition());
        telemetry.update();

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BNO055IMU.Parameters parameters1 = new BNO055IMU.Parameters();       //sets up IMU
        parameters1.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters1.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters1.calibrationDataFile = "REVHub1IMUCalibration.json";
        parameters1.loggingEnabled      = true;
        parameters1.loggingTag          = "IMU";

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters1);

        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        telemetry.addLine("ready to go!");
        telemetry.update();


        while (!a) {
            if (gamepad1.dpad_left) {
                --distance;
            }
            if (gamepad1.dpad_right) {
                ++distance;
            }
            if (gamepad1.dpad_up) {
                far = true;
            }
            if (gamepad1.dpad_down) {
                far = false;
            }
            if (gamepad1.right_bumper) {
                ++delay;
            }
            if (gamepad1.left_bumper) {
                --delay;
            }
            if (gamepad1.a) {
                a = true;
            }
            if (gamepad1.b) {
                left = false;
            }
            telemetry.addData("Distance:", distance);
            telemetry.addData("Delay:", delay);
            telemetry.addData("LF Starting Pos:", leftFront.getCurrentPosition());
            telemetry.addData("LB Starting Pos:", leftBack.getCurrentPosition());
            telemetry.addData("RF Starting Pos:", rightFront.getCurrentPosition());
            telemetry.addData("RB Starting Pos:", rightBack.getCurrentPosition());
            telemetry.addLine("D Pad Left/Right for Distance");
            telemetry.addLine("Bumpers for delay");
            telemetry.addLine("A to finish and move on");
            telemetry.update();
        }

        waitForStart();
        if (distance < 0) {
            distance = 0;
        }
        sleep(delay);
        encoderDrive(.25, distance, 31, false);
    }

    private void encoderDrive(double speed, double inches, double timeoutS, boolean strafe) {

        telemetry.addLine("Encoder Drive");

        int newLFTarget;
        int newRFTarget;
        int newLBTarget;
        int newRBTarget;
        int lFPos = leftFront.getCurrentPosition();
        int rFPos = rightFront.getCurrentPosition();
        int lBPos = leftBack.getCurrentPosition();
        int rBPos = rightBack.getCurrentPosition();
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, ZYX, AngleUnit.DEGREES);
        double startAngle = angles.firstAngle;

        if (opModeIsActive()) {
            if (strafe) {
                newLFTarget = lFPos + (int) (inches * ticksPerInch);
                newRFTarget = rFPos - (int) (inches * ticksPerInch);
                newLBTarget = lBPos - (int) (inches * ticksPerInch);
                newRBTarget = rBPos + (int) (inches * ticksPerInch);
            } else {
                newLFTarget = lFPos + (int) (inches * ticksPerInch);
                newRFTarget = rFPos + (int) (inches * ticksPerInch);
                newLBTarget = lBPos + (int) (inches * ticksPerInch);
                newRBTarget = rBPos + (int) (inches * ticksPerInch);
            }

            telemetry.addData("speed", speed);
            telemetry.addData("inches", inches);
            telemetry.addData("newLFTarget", newLFTarget);
            telemetry.addData("newRFTarget", newRFTarget);
            telemetry.addData("newLBTarget", newLBTarget);
            telemetry.addData("newRBTarget", newRBTarget);

            leftFront.setTargetPosition(newLFTarget);
            rightFront.setTargetPosition(newRFTarget);
            leftBack.setTargetPosition(newLBTarget);
            rightBack.setTargetPosition(newRBTarget);

            leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            leftFront.setPower(Math.abs(speed));
            rightFront.setPower(Math.abs(speed));
            leftBack.setPower(Math.abs(speed));
            rightBack.setPower(Math.abs(speed));

            runtime.reset();
            while (opModeIsActive() && runtime.seconds() < timeoutS && (leftFront.isBusy() && rightFront.isBusy() && leftBack.isBusy() && rightBack.isBusy())) {
                if (!strafe) {
                    double error = kP * (startAngle - angles.firstAngle);
                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, ZYX, AngleUnit.DEGREES);
                    telemetry.addData("Start Angle", startAngle);
                    telemetry.addData("Current Angle", angles.firstAngle);
                    telemetry.addData("error", error);
                    telemetry.update();
                    if (error > 0) {
                        leftFront.setPower(Math.abs(speed) - error);
                        rightFront.setPower(Math.abs(speed) + error);
                        leftBack.setPower(Math.abs(speed) - error);
                        rightBack.setPower(Math.abs(speed) + error);
                    } else if (error < 0) {
                        leftFront.setPower(Math.abs(speed) + error);
                        rightFront.setPower(Math.abs(speed) - error);
                        leftBack.setPower(Math.abs(speed) + error);
                        rightBack.setPower(Math.abs(speed) - error);
                    }
                }
                telemetry.addData("LF Current Position", leftFront.getCurrentPosition());
                telemetry.addData("RF Current Position", rightFront.getCurrentPosition());
                telemetry.addData("LB Current Position", leftBack.getCurrentPosition());
                telemetry.addData("RB Current Position", rightBack.getCurrentPosition());
                telemetry.addData("LF Current Power", leftFront.getPower());
                telemetry.addData("RF Current Power", rightFront.getPower());
                telemetry.addData("LB Current Power", leftBack.getPower());
                telemetry.addData("RB Current Power", rightBack.getPower());
                telemetry.update();
            }
            leftFront.setPower(0);
            rightFront.setPower(0);
            leftBack.setPower(0);
            rightBack.setPower(0);

            leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(100);
        }
    }

    private void gyroTurn(double targetAngle) {
        //+ is counter-clockwise
        //- is clockwise
        boolean finished = false;
        while (!finished) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, ZYX, AngleUnit.DEGREES);
            double currentAngle = angles.firstAngle;
            double e = (targetAngle - currentAngle);
            totalError += e;
            double error = (kP * e) - (kD * (currentAngle - lastAngle)) + (kI) * (totalError);
            lastAngle = currentAngle;
            leftFront.setPower(-error);
            rightFront.setPower(error);
            leftBack.setPower(-error);
            rightBack.setPower(error);
            telemetry.addData("targetAngle", targetAngle);
            telemetry.addData("currentAngle", currentAngle);
            telemetry.addData("error", error);
            telemetry.addData("targetAngle - currentAngle", targetAngle - currentAngle);
            telemetry.addData("finished", finished);



            telemetry.addData("LFM Current Power", leftFront.getPower());
            telemetry.addData("RFM Current Power", rightFront.getPower());
            telemetry.addData("LBM Current Power", leftBack.getPower());
            telemetry.addData("RBM Current Power", rightBack.getPower());

            telemetry.update();
            if (Math.abs(targetAngle - currentAngle) < 4) {
                finished = true;
                telemetry.addData("Finished", finished);
                rightFront.setPower(0);
                rightBack.setPower(0);
                leftFront.setPower(0);
                leftBack.setPower(0);
                sleep(500);
            }
        }
    }
}
