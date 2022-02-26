package org.firstinspires.ftc.teamcode; //Package class

/* IMPORT CLASSES NEEDED FOR FTC*/
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;


/* TELE-OP CLASS*/
@TeleOp(name = "SRJT04", group = "Opmode")
@Disabled
public class SRJT04 extends OpMode {

    // MOTORS
    private DcMotor flMotor; //front left motor
    private DcMotor frMotor; //front right motor
    private DcMotor blMotor; //back left motor
    private DcMotor brMotor; //back right motors
    private DcMotor liftMotor;
    private DcMotor tiltMotor;
    private DcMotor intakeMotor;

    // SERVOS
    private CRServo carouselRight;
    private CRServo carouselLeft;
    private Servo tiltServo;

    // SENSORS
    private RevBlinkinLedDriver blinkinLedDriver;
    private RevBlinkinLedDriver.BlinkinPattern pattern;


    // VARIABLES FOR DIRECTIONS
    private double forwardPower = 1;
    private double rotationPower = 1;

    liftPIDController liftPIDController;

    @Override
    public void init() {
        liftPIDController = new liftPIDController(0 , 0 , 0);
        //Motors for Tank Drive
        flMotor = hardwareMap.get(DcMotor.class, "flMotor");
        frMotor = hardwareMap.get(DcMotor.class, "frMotor");
        blMotor = hardwareMap.get(DcMotor.class, "blMotor");
        brMotor = hardwareMap.get(DcMotor.class, "brMotor");

        //Motors for intake
        liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        tiltMotor = hardwareMap.get(DcMotor.class, "tiltMotor");

        //servos
        carouselRight = hardwareMap.get(CRServo.class, "carouselRight");
        carouselLeft = hardwareMap.get(CRServo.class, "carouselLeft");
        tiltServo = hardwareMap.get(Servo.class, "tiltServo");

        // sensors
        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        lightStrip();

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
        liftMotor.setPower(0);
        // intakeMotor.setPower(0);
        tiltMotor.setPower(0);


        // Update Telemetry
        telemetry.addData("Status", "Started");
        telemetry.update();
    }
    @Override

    public void loop() {
        drive();
        intakeLift();
        carouselSpinner();
        telemetry.addData("position of lift", liftMotor.getCurrentPosition());
        telemetry.addData("servo position", tiltServo.getPosition());
        telemetry.update();
    }

    public void stop() {
        // Set motor power to 0
        flMotor.setPower(0);
        frMotor.setPower(0);
        blMotor.setPower(0);
        brMotor.setPower(0);
        liftMotor.setPower(0);
        tiltMotor.setPower(0);
    }

    public void tiltToPosition(double tiltTargetPosition , double movementSpeed) {
        double startTime = 0;
        double stopState = 0;
        while(stopState < 500){
            double power = liftPIDController.getOutput(liftMotor.getCurrentPosition() , tiltTargetPosition);
            tiltMotor.setPower(power);
            if(Math.abs(liftPIDController.getError()) < 2){
                stopState = System.currentTimeMillis() - startTime;
            }else{
                startTime = System.currentTimeMillis();
            }
        }
    }



////////////////////////////////////////////////////////////////////////////



    public void drive() {
        flMotor.setDirection(DcMotor.Direction.REVERSE);
        frMotor.setDirection(DcMotor.Direction.FORWARD);
        blMotor.setDirection(DcMotor.Direction.FORWARD);
        brMotor.setDirection(DcMotor.Direction.FORWARD);
        double xValue = gamepad1.right_stick_x * rotationPower;
        double yValue = gamepad1.left_stick_y * -forwardPower;
        double rValue = (gamepad1.left_trigger - gamepad1.right_trigger);

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

    public void intakeLift()  {
        tiltMotor.setDirection(DcMotor.Direction.FORWARD); //-185
        double tiltPower;
        double liftPower;
        telemetry.addData("tilt motor current position", tiltMotor.getCurrentPosition());
        telemetry.addData("lift motor current position", liftMotor.getCurrentPosition());


        //MOve up slightly
        //start
        //tilt: 169
        //lift: -324
        // slightly up
        //tilt: -45
        //lift: -628
        //Intake
        // tilt:
        // lift:
        // extends to first
        //sec
        //third
        // tilt: -394
        // lift: 758

        if (gamepad2.y)
        {
            liftPower = .80;
            tiltPower = .80;
            tiltMotor.setTargetPosition(-526);
            tiltMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftMotor.setTargetPosition(1065);
            liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // tiltServo.setPosition(.25);
        }

        if (gamepad1.dpad_up)
        {
            liftPower = .80;
            liftMotor.setTargetPosition(-314);
            liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftMotor.setPower(liftPower);
        }

        if (gamepad1.dpad_left)
        {
            tiltPower = .80;
            tiltMotor.setTargetPosition(-5);
            tiltMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            tiltMotor.setPower(tiltPower);
            tiltServo.setPosition(0.53);
        }

        if (gamepad1.dpad_down)
        {
            liftPower = .80;
            liftMotor.setTargetPosition(-122);
            liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftMotor.setPower(liftPower);
        }

        if (gamepad1.dpad_right)
        {
            tiltPower = .80;
            liftPower = .80;
            tiltMotor.setTargetPosition(0);
            tiltMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            tiltMotor.setPower(tiltPower);

            liftMotor.setTargetPosition(0);
            liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftMotor.setPower(liftPower);

        }



        if (gamepad2.dpad_up)
        {
            liftPower = .80;
            liftMotor.setTargetPosition(-342);
            liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftMotor.setPower(liftPower);

            // tiltServo.setPosition(.25);
        }

        if (gamepad2.dpad_right) {
            tiltPower = .80;
            tiltMotor.setTargetPosition(-272);
            tiltMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            tiltMotor.setPower(tiltPower);
            tiltServo.setPosition(0.25);
        }


        //INTAKE
        if (gamepad2.right_bumper) {
            intakeMotor.setPower(-.75);
        }

        if (gamepad2.left_bumper) {
            intakeMotor.setPower(0);
        }

        //TILT SERVO:
        if (gamepad2.dpad_left) {
            tiltServo.setPosition(0.25); // moving out
        }

        else if (gamepad2.dpad_down) {
            tiltServo.setPosition(0.75); //dropping
        }

    }

    public void lightStrip() {
        pattern = RevBlinkinLedDriver.BlinkinPattern.AQUA;
        blinkinLedDriver.setPattern(pattern);
        if (gamepad1.dpad_up) {
            pattern = RevBlinkinLedDriver.BlinkinPattern.GREEN;
            blinkinLedDriver.setPattern(pattern);
        }
    }

    public void carouselSpinner() {
        // 1a - 0; 1b - 1; carouselSpinner
        if (gamepad1.a) {
            carouselRight.setPower(0.0);
            carouselLeft.setPower(0.0);
        } else if (gamepad1.b) {
            carouselRight.setPower(1.0);
            carouselLeft.setPower(1.0);
        } else if (gamepad1.x) {
            carouselRight.setPower(-1.0);
            carouselLeft.setPower(-1.0);
        }
    }
} //End Brace