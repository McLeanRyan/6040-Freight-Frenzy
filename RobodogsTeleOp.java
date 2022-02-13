package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class RobodogsTeleOp extends OpMode {
    public DcMotor leftFront;
    public DcMotor leftBack;
    public DcMotor rightFront;
    public DcMotor rightBack;

    public DcMotor intake;
    public DcMotor lift;

    public CRServo duck;
    public CRServo box;

    @Override
    public void init() {
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        intake = hardwareMap.get(DcMotor.class, "intake");
        lift = hardwareMap.get(DcMotor.class, "lift");

        duck = hardwareMap.get(CRServo.class, "duck");
        box = hardwareMap.get(CRServo.class, "box");

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addLine("Initialized...");
        telemetry.addLine("Press Play to Start!");
    }

    @Override
    public void loop() {
        double px = gamepad1.left_stick_x;
        if (Math.abs(px) < 0.05) px = 0;
        double py = -gamepad1.left_stick_y;
        if (Math.abs(py) < 0.05) py = 0;
        double pa = -(gamepad1.right_stick_x*(.70));
        if (Math.abs(pa) < 0.05) pa = 0;
        double plf = -px + py - pa;
        double plb = px + py + -pa;
        double prf = -px + py + pa;
        double prb = px + py + pa;
        double max = Math.max(1.0, Math.abs(plf));
        max = Math.max(max, Math.abs(plb));
        max = Math.max(max, Math.abs(prf));
        max = Math.max(max, Math.abs(prb));
        plf /= max;
        plb /= max;
        prf /= max;
        prb /= max;
        leftFront.setPower(plf);
        leftBack.setPower(plb);
        rightFront.setPower(prf);
        rightBack.setPower(prb);

        while (gamepad1.right_trigger > 0)    {
            leftFront.setPower(1);
            leftBack.setPower(-1);
            rightFront.setPower(-1);
            rightBack.setPower(1);
        }

        while (gamepad1.left_trigger > 0)       {
            leftFront.setPower(-1);
            leftBack.setPower(1);
            rightFront.setPower(1);
            rightBack.setPower(-1);
        }

        while (gamepad1.right_bumper)    {
            leftFront.setPower(0.5);
            leftBack.setPower(-0.5);
            rightFront.setPower(-0.5);
            rightBack.setPower(0.5);
        }

        while (gamepad1.left_bumper)       {
            leftFront.setPower(-0.5);
            leftBack.setPower(0.5);
            rightFront.setPower(0.5);
            rightBack.setPower(-0.5);
        }

        lift.setPower(-gamepad2.left_stick_y * 0.75);

        if(gamepad2.left_bumper) {
            duck.setPower(1);
        } else if (gamepad2.right_bumper) {
            duck.setPower(-1);
        }else {
            duck.setPower(0);
        }

        if (gamepad2.right_trigger > 0.15) {
            intake.setPower(1);
        } else if (gamepad2.left_trigger > 0.15) {
            intake.setPower(-1);
        } else {
            intake.setPower(0);
        }

        box.setPower(-gamepad2.right_stick_y/3);
    }
}
