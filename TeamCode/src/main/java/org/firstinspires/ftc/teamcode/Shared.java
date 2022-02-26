package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Shared", group = "Opmode")

public class Shared extends OpMode {
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
    RevColorSensorV3 colorSensor2;
    RevBlinkinLedDriver blinkinLedDriver;
    RevBlinkinLedDriver.BlinkinPattern pattern;

    private double forwardPower = 1;
    private double rotationPower = 1;
    liftPIDController liftPIDController;
    private ElapsedTime carouselTimer;
    private ElapsedTime turnTimer;
    double weightedDrivePower;

    @Override
    public void init() {
        liftPIDController = new liftPIDController(0 , 0 , 0);
        flMotor = hardwareMap.get(DcMotor.class, "flMotor");
        frMotor = hardwareMap.get(DcMotor.class, "frMotor");
        blMotor = hardwareMap.get(DcMotor.class, "blMotor");
        brMotor = hardwareMap.get(DcMotor.class, "brMotor");
        liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        tiltMotor = hardwareMap.get(DcMotor.class, "tiltMotor");
        carouselRight = hardwareMap.get(CRServo.class, "carouselRight");
        carouselLeft = hardwareMap.get(CRServo.class, "carouselLeft");
        tiltServo = hardwareMap.get(Servo.class, "tiltServo");
        dropServo = hardwareMap.get(Servo.class, "dropServo");
        colorSensor = hardwareMap.get(RevColorSensorV3.class, "sensor_color");
        colorSensor2 = hardwareMap.get(RevColorSensorV3.class, "sensor_color2");
        carouselTimer = new ElapsedTime();
        turnTimer = new ElapsedTime();
        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        tiltMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void start() {
        flMotor.setPower(0);
        frMotor.setPower(0);
        blMotor.setPower(0);
        brMotor.setPower(0);
        liftMotor.setPower(0);
        tiltMotor.setPower(0);
    }
    @Override
    public void loop() {
        drive();
        intake();
        lift();
        carouselSpinner();
        detectFreight();
        logTelemetry();
    }

    public void stop() {
        flMotor.setPower(0);
        frMotor.setPower(0);
        blMotor.setPower(0);
        brMotor.setPower(0);
        intakeMotor.setPower(0);
        liftMotor.setPower(0);
        tiltMotor.setPower(0);
    }

    public void drive() {

        double y = -gamepad1.left_stick_y * weightedDrivePower ; // Remember, this is reversed!
        double x = gamepad1.left_stick_x * 1.1 * weightedDrivePower; // Counteract imperfect strafing
        double rx = gamepad1.right_stick_x * weightedDrivePower;

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        flMotor.setPower(frontLeftPower);
        blMotor.setPower(backLeftPower);
        frMotor.setPower(frontRightPower);
        brMotor.setPower(backRightPower);

        flMotor.setDirection(DcMotor.Direction.REVERSE);
        frMotor.setDirection(DcMotor.Direction.FORWARD);
        blMotor.setDirection(DcMotor.Direction.FORWARD);
        brMotor.setDirection(DcMotor.Direction.FORWARD);


        /*This if statement uses the inverse of speed which is torque
         *by constantly checking if the tilt motor is beyond a certain threshold it'll slow down the motors
         * we do this because if the arm is tilting upwards that means we are about to go over the barrier which means
         * we want to go over the barrier at a slow controlled pace. Otherwise it should move with speed.
         */
        if (tiltMotor.getCurrentPosition() < -300 ) {
            weightedDrivePower = 0.65;
        }
        else {
            weightedDrivePower = 0.65;
        }


    }

    /*
     *The lift() method allows for us to bind buttons for each stage of the shipping hub
     * as well as reset the lift to the stage zero position
     *To do so:
     *  A - brings the arm up *Note: This should automatically be triggered by the ColorSensor*
     *  Y - brings the arm to the third level
     *  X - brings the arm all the way in
     *  B - brings the arm all the way down and resets lights and ready's the robot to start intaking
     */
    public void lift() {
        tiltMotor.setDirection(DcMotor.Direction.FORWARD);
        if (gamepad2.x) {
            dropServo.setPosition(1);
            liftMotor.setTargetPosition(0);
            liftMotor.setPower(0.8);
            liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            tiltServo.setPosition(.50);
        }
        if (gamepad2.b) {
            tiltMotor.setTargetPosition(-9);
            tiltMotor.setPower(.35);
            tiltMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            dropServo.setPosition(1);
            pattern = RevBlinkinLedDriver.BlinkinPattern.RED;
            blinkinLedDriver.setPattern(pattern);
        }
        if (gamepad2.a) {
            tiltMotor.setTargetPosition(-272);
            tiltMotor.setPower(.6);
            tiltMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftMotor.setTargetPosition(-7);
            liftMotor.setPower(0.5);
            liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            tiltServo.setPosition(.15);
            dropServo.setPosition(1);
        }
        if (gamepad2.y) {
            intakeMotor.setPower(0);
            tiltMotor.setTargetPosition(-375);
            tiltMotor.setPower(.6);
            tiltMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftMotor.setTargetPosition(900);
            liftMotor.setPower(0.8);
            liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            tiltServo.setPosition(.15);
            dropServo.setPosition(1);
        }
        if (gamepad2.dpad_down) {
            dropServo.setPosition(0);
        }
    }

    /*
     *The intake() method allows the driver to turn on and off the intake by pressing
     *the left and right bumpers.
     * Also, in case of a jam the driver can push down on the left bumper if freight gets stuck
     * which will reverse the intake motor and unjam the robot.
     */
    public void intake() {
        if (gamepad2.right_bumper) {
            intakeMotor.setPower(-.89);
        }
        if (gamepad2.left_bumper) {
            intakeMotor.setPower(0);
        }
        if (gamepad2.left_stick_button) {
            intakeMotor.setPower(.75);
        }
    }

    /* detectFreight()
     * We have two ColorSensor V3's. One is ColorSensor and is mounted directly under the intake and
     * colorSensor2 is mounted on the top of the bucket and pointed to the inside. If ColorSensor2
     * detects freight it will automatically lift up the arm as well as turn the lights green. If
     * freight has entered meaning the ColorSensor2 distance is below a certain threshold AND the
     * colorSensor detects freight it will then reverse the intake so that we do not hold two at a time.
     */
    public void detectFreight() {

        if (colorSensor2.getDistance(DistanceUnit.MM) < 35 && intakeMotor.getPower() < -.5 || intakeMotor.getPower() > .5) {
            //Run A Command
            tiltMotor.setTargetPosition(-272);
            tiltMotor.setPower(.6);
            tiltMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftMotor.setTargetPosition(-7);
            liftMotor.setPower(0.5);
            liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            tiltServo.setPosition(.15);
            dropServo.setPosition(1);
            //End of A Command

            intakeMotor.setPower(.70);
            //Control Lights
            pattern = RevBlinkinLedDriver.BlinkinPattern.GREEN;
            blinkinLedDriver.setPattern(pattern);
            //End of Light Control
        }

        if (colorSensor.getDistance(DistanceUnit.MM) < 35 && colorSensor2.getDistance(DistanceUnit.MM ) < 35) {
            intakeMotor.setPower(.60); //reverses intake
        }
    }

    /*
     *The carouselSpinner() method let's us spin our 72mm Gecko wheels at 2 second intervals
     *When the operator presses left on the dpad then it spins for 2500ms, stops and then starts up again
     */
    public void carouselSpinner() {
        carouselRight.setDirection(DcMotorSimple.Direction.REVERSE);
        carouselLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        if (gamepad1.dpad_left) {
            if (carouselTimer.milliseconds() <= 2500) {
                carouselLeft.setPower(1.0);
                carouselRight.setPower(1.0);
            }
            else if (carouselTimer.milliseconds() > 2500 && carouselTimer.milliseconds() <= 3250){
                carouselLeft.setPower(0.0);
                carouselRight.setPower(0.0);
            } else {
                carouselTimer.reset();
            }
        }

        if (gamepad1.dpad_right) {
            carouselLeft.setPower(0.0);
            carouselRight.setPower(0.0);
        }
    }

    /*
     *The logTelemetry() method prints a string value, and an object value to the Driver Station
     *The name convention that 16606 follows is "DataType: sentence of what is need : and then the object
     */
    public void logTelemetry() {
        telemetry.addData("MOTOR: tiltMotor current position: ", tiltMotor.getCurrentPosition());
        telemetry.addData("MOTOR: liftMotor current position: ", liftMotor.getCurrentPosition());
        telemetry.addData("MOTOR: intakeMotor power: ", intakeMotor.getPower());
        telemetry.addData("SERVO: tiltServo current position: ", tiltServo.getPosition());
        telemetry.addData("SENSOR: colorSensor: ", colorSensor.getDistance(DistanceUnit.MM));
        telemetry.addData("SENSOR: colorSensor2: ", colorSensor2.getDistance(DistanceUnit.MM));
        telemetry.addData("TIMER: carouselTimer: ", carouselTimer.milliseconds());
    }

} //End Brace