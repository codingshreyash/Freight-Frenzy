package org.firstinspires.ftc.teamcode; //Package class

/* IMPORT CLASSES NEEDED FOR FTC*/
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

/* TELE-OP CLASS*/
@TeleOp(name = "ACJT01", group = "Opmode")
@Disabled
public class ACJT01 extends OpMode {

    // MOTORS
    private DcMotor flMotor; //front left motor
    private DcMotor frMotor; //front right motor
    private DcMotor blMotor; //back left motor
    private DcMotor brMotor; //back right motor

    // VARIABLES FOR DIRECTIONS
    private double forwardPower = 1;
    private double rotationPower = 1;
    private double strafePower = 1;

    @Override
    public void init() {
        flMotor = hardwareMap.get(DcMotor.class, "flMotor");
        frMotor = hardwareMap.get(DcMotor.class, "frMotor");
        blMotor = hardwareMap.get(DcMotor.class, "blMotor");
        brMotor = hardwareMap.get(DcMotor.class, "brMotor");

        // Update status
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    public void start() {
        // Set motor power to 0
        flMotor.setPower(0);
        frMotor.setPower(0);
        blMotor.setPower(0);
        brMotor.setPower(0);

        // Update Telemetry
        telemetry.addData("Status", "Started");
        telemetry.update();
    }
    @Override
    public void loop() {
        drive();
    }
    public void stop() {
        // Set motor power to 0
        flMotor.setPower(0);
        frMotor.setPower(0);
        blMotor.setPower(0);
        brMotor.setPower(0);
        // Update Telemetry
        telemetry.addData("Status", "Stopped");
        telemetry.update();
    }
    // DRIVE TRAIN CODE
    public void drive() {
        flMotor.setDirection(DcMotor.Direction.FORWARD);
        frMotor.setDirection(DcMotor.Direction.REVERSE);
        blMotor.setDirection(DcMotor.Direction.FORWARD);
        brMotor.setDirection(DcMotor.Direction.REVERSE);
        double xValue = gamepad1.right_stick_x * -rotationPower;
        double yValue = gamepad1.left_stick_y * forwardPower;
        double rValue = (gamepad1.left_trigger - gamepad1.right_trigger) * strafePower;

        double flPower = yValue + xValue + rValue;
        double frPower = yValue - xValue - rValue;
        double blPower = yValue + xValue - rValue;
        double brPower = yValue - xValue + rValue;

        telemetry.addData("Y Value", yValue);

        flMotor.setPower(Range.clip(Math.signum(flPower) * Math.pow(flPower, 2), -1.0, 1.0));
        frMotor.setPower(Range.clip(Math.signum(frPower) * Math.pow(frPower, 2), -1.0, 1.0));
        blMotor.setPower(Range.clip(Math.signum(blPower) * Math.pow(blPower, 2), -1.0, 1.0));
        brMotor.setPower(Range.clip(Math.signum(brPower) * Math.pow(brPower, 2), -1.0, 1.0));

    }
}