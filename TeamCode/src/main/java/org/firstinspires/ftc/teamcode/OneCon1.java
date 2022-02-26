package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "OneCon1", group = "Opmode")
public class OneCon1 extends OpMode {
    private DcMotor flMotor;
    private DcMotor frMotor;
    private DcMotor blMotor;
    private DcMotor brMotor;
    private DcMotor liftMotor;
    private DcMotor tiltMotor;
    private DcMotor intakeMotor;
    private CRServo carouselRight;
    private CRServo carouselLeft;
    private Servo tiltServo;
    private Servo dropServo;
    RevColorSensorV3 colorSensor;
    //DigitalChannel touchSensor;
    RevBlinkinLedDriver blinkinLedDriver;
    RevBlinkinLedDriver.BlinkinPattern pattern;


    // VARIABLES FOR DIRECTIONS
    private double forwardPower = 1;
    private double rotationPower = 1;
    liftPIDController liftPIDController;
    private ElapsedTime carouselTimer;
    private ElapsedTime turnTimer;

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
        dropServo = hardwareMap.get(Servo.class, "dropServo");
        colorSensor = hardwareMap.get(RevColorSensorV3.class, "sensor_color");
        carouselTimer = new ElapsedTime();
        turnTimer = new ElapsedTime();

        // sensors
//        touchSensor = hardwareMap.get(DigitalChannel.class, "sensor_touch");
//        touchSensor.setMode(DigitalChannel.Mode.INPUT);
        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        tiltMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void start() {
        // Set motor power to 0
        flMotor.setPower(0);
        frMotor.setPower(0);
        blMotor.setPower(0);
        brMotor.setPower(0);
        liftMotor.setPower(0);
        tiltMotor.setPower(0);
        telemetry.addData("Status", "Started");
        telemetry.update();
    }
    @Override

    public void loop() {
        drive();
        intakeLift();
        carouselSpinner();
        detectFreight();
        logTelemetry();
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

    /*
     *The regular distance the colorSensor detects is something like 6.5
     *So, if the color sensor detects something less than 5.5 meaning freight has entered
     *then, set the intake to run the opposite direction & start to lift arm up.
     */
    public void detectFreight() {
        if (colorSensor.getDistance(DistanceUnit.CM) < 5.5) {

            intakeMotor.setPower(.60); //reverses intake
            //Run A Command
            tiltMotor.setTargetPosition(-216); tiltMotor.setPower(.6); tiltMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            liftMotor.setTargetPosition(-7); liftMotor.setPower(0.6); liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            tiltServo.setPosition(.23); dropServo.setPosition(1);
            //End of A Command

            //Control Lights
            pattern = RevBlinkinLedDriver.BlinkinPattern.GREEN; blinkinLedDriver.setPattern(pattern);
            //End of Light Control
        }
        else {
            pattern = RevBlinkinLedDriver.BlinkinPattern.RED; blinkinLedDriver.setPattern(pattern);
        }
    }

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

        flMotor.setPower(Range.clip(Math.signum(flPower) * Math.pow(flPower, 2), -1.0, 1.0));
        frMotor.setPower(Range.clip(Math.signum(frPower) * Math.pow(frPower, 2), -1.0, 1.0));
        blMotor.setPower(Range.clip(Math.signum(blPower) * Math.pow(blPower, 2), -1.0, 1.0));
        brMotor.setPower(Range.clip(Math.signum(brPower) * Math.pow(brPower, 2), -1.0, 1.0));

        if (gamepad2.a) {
            // rotate 60 degrees right
            if (turnTimer.milliseconds() <= 150) {
                flMotor.setPower(rotationPower);
                blMotor.setPower(rotationPower);
                frMotor.setPower(-rotationPower);
                brMotor.setPower(-rotationPower);
            }
            else if (turnTimer.milliseconds() > 150 && carouselTimer.milliseconds() <= 3250){
                flMotor.setPower(0);
                blMotor.setPower(0);
                frMotor.setPower(0);
                brMotor.setPower(0);
            } else {
                turnTimer.reset(); //set timer to 0
            }
        }

        if (gamepad2.b) {
            // rotate 60 degrees left
            if (carouselTimer.milliseconds() <= 150){
                flMotor.setPower(-rotationPower);
                blMotor.setPower(-rotationPower);
                frMotor.setPower(rotationPower);
                brMotor.setPower(rotationPower);
            }
            else if (carouselTimer.milliseconds() > 150 && carouselTimer.milliseconds() <= 3250){
                flMotor.setPower(0);
                blMotor.setPower(0);
                frMotor.setPower(0);
                brMotor.setPower(0);
            } else {
                turnTimer.reset();
            }
        }
    }

    public void intakeLift()  { //intakeMethod
        tiltMotor.setDirection(DcMotor.Direction.FORWARD); //-185

        if(gamepad1.x) {
            dropServo.setPosition(1);
            liftMotor.setTargetPosition(-7);
            liftMotor.setPower(0.4);
            liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        if(gamepad1.b) {
            tiltServo.setPosition(.78);
            tiltMotor.setTargetPosition(1);
            tiltMotor.setPower(.35);
            tiltMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            dropServo.setPosition(1);

        }
        if(gamepad1.a){
            tiltMotor.setTargetPosition(-216);
            tiltMotor.setPower(.6);
            tiltMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftMotor.setTargetPosition(-7);
            liftMotor.setPower(0.6);
            liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            tiltServo.setPosition(.23);
            dropServo.setPosition(1);
        }

        if(gamepad1.y){
            tiltMotor.setTargetPosition(-510);
            tiltMotor.setPower(.7);
            tiltMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftMotor.setTargetPosition(1247);
            liftMotor.setPower(0.40);
            liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            tiltServo.setPosition(.15);
            dropServo.setPosition(1);
        }

        if (gamepad1.dpad_down) {
            dropServo.setPosition(0);
        }
        //INTAKE
        if (gamepad1.right_bumper) {
            intakeMotor.setPower(-.60);
        }
        if (gamepad1.left_bumper) {
            intakeMotor.setPower(0);
        }
        if (gamepad1.left_stick_button) {
            intakeMotor.setPower(.60);
        }
    }

    public void carouselSpinner() {
        carouselLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        // 1a - 0; 1b - 1; carouselSpinner
        if (gamepad2.dpad_left) {
            if (carouselTimer.milliseconds() <= 2500) //2 seconds
                carouselLeft.setPower(1.0); //power 1
            else if (carouselTimer.milliseconds() > 2500 && carouselTimer.milliseconds() <= 3250){ //(time to place) if time is more than 2 sec and less then 2.5.seconds
                carouselLeft.setPower(0.0); //setPower to 0
            } else {
                carouselTimer.reset(); //set timer to 0
            }
        }
    }

    public void rotate60(double rotatePower) {

        turnTimer.reset();

        while (turnTimer.milliseconds() <= time) {
            // telemetry.addData("current time", runtime.milliseconds());
            flMotor.setPower(rotatePower);
            blMotor.setPower(rotatePower);
            frMotor.setPower(-rotatePower);
            brMotor.setPower(-rotatePower);
        }
        flMotor.setPower(0);
        blMotor.setPower(0);
        frMotor.setPower(0);
        brMotor.setPower(0);
    }

    public void logTelemetry() {
        telemetry.addData("MOTOR: tiltMotor current position: ", tiltMotor.getCurrentPosition());
        telemetry.addData("MOTOR: liftMotor current position: ", liftMotor.getCurrentPosition());
        telemetry.addData("SERVO: tiltServo current position: ", tiltServo.getPosition());
        telemetry.addData("SENSOR: DISTANCE: ", colorSensor.getDistance(DistanceUnit.CM));
    }
} //End Brace