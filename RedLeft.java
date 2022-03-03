package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.opencv.core.Core;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Point;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "RedLeft", group = "")
public class RedLeft extends LinearOpMode {

    private DcMotor flMotor;
    private DcMotor frMotor;
    private DcMotor blMotor;
    private DcMotor brMotor;
    private DcMotor liftMotor;
    private DcMotor intakeMotor;
    private DcMotor tiltMotor;
    private Servo tiltServo;
    RevColorSensorV3 colorSensor2;
    private Servo dropServo;
    private CRServo carouselRight;
    private CRServo carouselLeft;
    private ElapsedTime runtime = new ElapsedTime();
    private BNO055IMU imu;
    Orientation threeAngles;
    boolean droveAlready;
    boolean rotatedAlready;
    Orientation lastAngles = new Orientation();
    private BarcodePosition barcodePosition = BarcodePosition.NOT_FOUND;
    NormalizedColorSensor colorSensor;
    VoltageSensor voltageSensor;

    private double voltageMult = 0.87571429;

    public static enum BarcodePosition {
        LEFT { @Override public String toString() {
            return "Left";
        } },
        MIDDLE { @Override public String toString() {
            return "Middle";
        } },
        RIGHT { @Override public String toString() {
            return "Right";
        } },
        NOT_FOUND { @Override public String toString() {
            return "Not found";
        }}
    }
    Mat mat = new Mat();
    static final Rect LEFT_ROI = new Rect(
            new Point(0, 144),
            new Point(90, 72)
    );
    static final Rect MIDDLE_ROI = new Rect(
            new Point(115, 144),
            new Point(205, 72)
    );
    static final Rect RIGHT_ROI = new Rect(
            new Point(230, 144),
            new Point(320, 72)
    );
    static double PERCENT_COLOR_THRESHOLD = 0.02;
    public void runOpMode() throws InterruptedException {
        colorSensor2 = hardwareMap.get(RevColorSensorV3.class, "sensor_color2");
        flMotor = hardwareMap.dcMotor.get("flMotor");
        flMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frMotor = hardwareMap.dcMotor.get("frMotor");
        frMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        blMotor = hardwareMap.dcMotor.get("blMotor");
        blMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        brMotor = hardwareMap.dcMotor.get("brMotor");
        brMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor = hardwareMap.dcMotor.get("intakeMotor");
        liftMotor = hardwareMap.get(DcMotorEx.class, "liftMotor");
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        tiltMotor = hardwareMap.get(DcMotorEx.class, "tiltMotor");
        tiltMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        tiltServo = hardwareMap.get(Servo.class, "tiltServo");
        dropServo = hardwareMap.get(Servo.class, "dropServo");
        carouselRight = hardwareMap.get(CRServo.class, "carouselRight");
        carouselLeft = hardwareMap.get(CRServo.class, "carouselLeft");
        imu = hardwareMap.get(BNO055IMU.class, "imu 1");
        threeAngles = new Orientation();
        droveAlready = false;
        rotatedAlready = false;
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(parameters);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        tiltMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");
        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        waitForStart(); //Pauses the Linear Op Mode until start has been pressed or until the current thread is interrupted.
        if (barcodePosition.equals(BarcodePosition.NOT_FOUND)) {
            telemetry.addData("voltage", voltageSensor.getVoltage());
            telemetry.update();

            voltageMult = 12.66 / voltageSensor.getVoltage();
            //MUST BE THERE START
            aCommand();
            moveForward(450, 40 * voltageMult);
            sleep(150);
            moveForward(420, 10 * voltageMult);
            CARASHREYASH(3500,5 * voltageMult);
            moveBackward(1000, 60 * voltageMult);
            yTopCommand();
            strafeLeft(450, 75 * voltageMult); //    10.8 inches
            rotateRight(360, 72 * 0.78 * voltageMult) ; // MATCH
            moveForward(530, 40 * voltageMult);
            openDrop();
            sleep(850);
            closeDrop();
            a1Command();
            moveBackward(500, 40 * voltageMult);
            rotateRight(300, -60 * 0.75 * voltageMult); //MATCH
            strafeLeft(700, -75 * voltageMult);

            //MUST BE THERE END

            //REPEAT THIS
            moveBackward(500, 50 * voltageMult);
            xCommand();
            sleep(100);
            bCommand();
            sleep(300);

            intakeMove(3000,25 * voltageMult); //the time here is rlly inaccruate idk but this works
            aCommand();
            SpitOutIntakeMove(175,60 * voltageMult);
            yTopCommand();
            strafeLeft(230, -40 * voltageMult);
            SpitOutIntakeMove(600, 70 * voltageMult);
            strafeLeft(260, -40 * voltageMult);
            moveForward(450, 40 * voltageMult);
            strafeLeft(450, 40 * voltageMult);
            rotateRight(515, 72 * 0.85 * voltageMult); // MATCH
            moveForward(570, 40 * voltageMult);
            openDrop();
            sleep(850);
            closeDrop();
            a1Command();
            moveBackward(570, 40 * voltageMult);
            rotateRight(380, -60 * 0.75 * voltageMult); //MATCH
            strafeLeft(700, -75 * voltageMult);
            moveBackward(850, 50 * voltageMult);
            xCommand();
            bCommand();
            sleep(200);
        }
    }
    ///////////////////////
    //COMMANDS USED ABOVE//
    //////////////////////
    private void moveForward(double time, double drivePower) {
        drivePower /= 100;
        flMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        blMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        brMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        brMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        flMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        blMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        runtime.reset();
        while (runtime.milliseconds() <= time) {
            flMotor.setPower(drivePower);
            blMotor.setPower(drivePower);
            frMotor.setPower(drivePower);
            brMotor.setPower(drivePower);
        }
        flMotor.setPower(0);
        blMotor.setPower(0);
        frMotor.setPower(0);
        brMotor.setPower(0);
        //sleep(500);
    }

    private void CARASHREYASH(double time, double drivePower) {

        drivePower /= 100;
        flMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        blMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        brMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        brMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        flMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        blMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        runtime.reset();
        while (runtime.milliseconds() <= time) {
            flMotor.setPower(drivePower);
            blMotor.setPower(drivePower);
            frMotor.setPower(drivePower);
            brMotor.setPower(drivePower);
            carouselRight.setPower(1);
            carouselLeft.setPower(1);
        }
        flMotor.setPower(0);
        blMotor.setPower(0);
        frMotor.setPower(0);
        brMotor.setPower(0);
        carouselRight.setPower(0);
        carouselLeft.setPower(0);
        //sleep(500);
    }
    private void SpitOutIntakeMove(double time, double drivePower) {
        intakeMotor.setPower(.75);
        drivePower /= 100;
        flMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        blMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        brMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        brMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        flMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        blMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        runtime.reset();
        while (runtime.milliseconds() <= time) {
            flMotor.setPower(drivePower);
            blMotor.setPower(drivePower);
            frMotor.setPower(drivePower);
            brMotor.setPower(drivePower);
        }
        flMotor.setPower(0);
        blMotor.setPower(0);
        frMotor.setPower(0);
        brMotor.setPower(0);
        //sleep(500);
    }

    public void openDrop(){
        dropServo.setPosition(0);
    }

    public void closeDrop(){
        dropServo.setPosition(1);
    }

    private void moveBackward(double time, double drivePower) {
        drivePower /= 100;
        flMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        blMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        brMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        brMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        flMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        blMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        runtime.reset();
        while (runtime.milliseconds() <= time) {
            flMotor.setPower(-drivePower);
            blMotor.setPower(-drivePower);
            frMotor.setPower(-drivePower);
            brMotor.setPower(-drivePower);
        }
        flMotor.setPower(0);
        blMotor.setPower(0);
        frMotor.setPower(0);
        brMotor.setPower(0);
        sleep(500);
    }

    private void intakeMove(double time, double drivePower) {
        drivePower /= 100;
        flMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        blMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        brMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        brMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        flMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        blMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        runtime.reset();


        while (colorSensor2.getDistance(DistanceUnit.MM) >= 35 || runtime.milliseconds() <= time) {
            intakeMotor.setPower(-.75);
            flMotor.setPower(-drivePower);
            blMotor.setPower(-drivePower);
            frMotor.setPower(-drivePower);
            brMotor.setPower(-drivePower);
        }

        flMotor.setPower(0);
        blMotor.setPower(0);
        frMotor.setPower(0);
        brMotor.setPower(0);
        sleep(500);

    }

    private void rotateRight(double time, double rotatePower) {
        rotatePower /= 100;
        flMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        blMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        brMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        brMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        flMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        blMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        runtime.reset();
        while (runtime.milliseconds() <= time) {
            flMotor.setPower(rotatePower);
            blMotor.setPower(rotatePower);
            frMotor.setPower(-rotatePower);
            brMotor.setPower(-rotatePower);
        }
        flMotor.setPower(0);
        blMotor.setPower(0);
        frMotor.setPower(0);
        brMotor.setPower(0);
        sleep(500);
    }

    private void strafeLeft(double time, double strafePower) {
        strafePower /= 100;
        flMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        blMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        brMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        brMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        flMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        blMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        runtime.reset();
        while (runtime.milliseconds() <= time) {
            flMotor.setPower(strafePower);
            blMotor.setPower(-strafePower);
            frMotor.setPower(-strafePower);
            brMotor.setPower(strafePower);
        }
        flMotor.setPower(0);
        blMotor.setPower(0);
        frMotor.setPower(0);
        brMotor.setPower(0);
        sleep(500);
    }

    private void strafeRight(double time, double strafePower) {
        strafePower /= 100;
        flMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        blMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        brMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        brMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        flMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        blMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        runtime.reset();
        while (runtime.milliseconds() <= time) {
            flMotor.setPower(strafePower);
            blMotor.setPower(strafePower);
            frMotor.setPower(-strafePower);
            brMotor.setPower(strafePower);
        }
        flMotor.setPower(0);
        blMotor.setPower(0);
        frMotor.setPower(0);
        brMotor.setPower(0);
        sleep(500);
    }

    public void spinWheelBlue(long time) {
        carouselRight.setDirection(DcMotorSimple.Direction.FORWARD);
        carouselLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        carouselRight.setPower(1);
        carouselLeft.setPower(1);
        sleep(time);
        carouselRight.setPower(0);
        carouselLeft.setPower(0);




    }

    public void xCommand() {
        dropServo.setPosition(1);
        liftMotor.setTargetPosition(0);
        liftMotor.setPower(0.8);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        tiltServo.setPosition(.45);
    }

    public void a1Command() {
        dropServo.setPosition(1);
        liftMotor.setTargetPosition(-12);
        liftMotor.setPower(0.65);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        tiltServo.setPosition(.50);
    }

    public void bCommand() {
        tiltMotor.setTargetPosition(-9);
        tiltMotor.setPower(.35);
        tiltMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        dropServo.setPosition(1);
    }

    public void aCommand() {
        tiltMotor.setTargetPosition(-272);
        tiltMotor.setPower(.6);
        tiltMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setTargetPosition(-7);
        liftMotor.setPower(0.5);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        tiltServo.setPosition(.15);
        dropServo.setPosition(1);
    }

    public void yTopCommand() {
        tiltMotor.setTargetPosition(-486);
        tiltMotor.setPower(.7);
        tiltMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setTargetPosition(1700);
        liftMotor.setPower(0.8);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        tiltServo.setPosition(.15);
        dropServo.setPosition(1);
    }

    public class DuckDetectionPipeline extends OpenCvPipeline
    { public Mat processFrame(Mat input, String type) {
        System.out.println(input.size());
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        Scalar lowHSV;
        Scalar highHSV;

        if (type.equalsIgnoreCase( "duck" )) {
            lowHSV = new Scalar(25, 25, 35);
            highHSV = new Scalar(40, 255, 255);
        } else if (type.equalsIgnoreCase( "custom" )) {
            lowHSV = new Scalar(54, 82, 67);
            highHSV = new Scalar(181, 192, 150);
        } else {
            lowHSV = new Scalar(40, 50, 70);
            highHSV = new Scalar(65, 255, 255);
        }
        Core.inRange(mat, lowHSV, highHSV, mat);

        Mat left = mat.submat(LEFT_ROI);
        Mat middle = mat.submat(MIDDLE_ROI);
        Mat right = mat.submat(RIGHT_ROI);

        double leftValue = Core.sumElems( left ).val[0] / LEFT_ROI.area( ) / 255;
        double middleValue = Core.sumElems( middle ).val[0] / MIDDLE_ROI.area( ) / 255;
        double rightValue = Core.sumElems( right ).val[0] / RIGHT_ROI.area( ) / 255;

        left.release( );
        middle.release( );
        right.release( );

        boolean leftBool = leftValue > PERCENT_COLOR_THRESHOLD;
        boolean middleBool = middleValue > PERCENT_COLOR_THRESHOLD;
        boolean rightBool = rightValue > PERCENT_COLOR_THRESHOLD;

        if( rightBool ) {
            barcodePosition = BarcodePosition.RIGHT;
        } else if( leftBool ) {
            barcodePosition = BarcodePosition.LEFT;
        } else if( middleBool ) {
            barcodePosition = BarcodePosition.MIDDLE;
        } else {
            barcodePosition = BarcodePosition.NOT_FOUND;
        }
        Imgproc.cvtColor( mat, mat, Imgproc.COLOR_GRAY2RGB );

        Scalar elementColor = new Scalar( 255, 0 );
        Scalar notElement = new Scalar( 0, 255, 0 );

        Imgproc.rectangle( mat, LEFT_ROI, barcodePosition == BarcodePosition.LEFT ? notElement : elementColor );
        Imgproc.rectangle( mat, RIGHT_ROI, barcodePosition == BarcodePosition.RIGHT ? notElement : elementColor );
        Imgproc.rectangle( mat, MIDDLE_ROI, barcodePosition == BarcodePosition.MIDDLE ? notElement : elementColor );
        System.out.println("location: " + barcodePosition);
        return mat;
    } @Override public Mat processFrame(Mat input) {
        Mat elementImage = processFrame( input, "element" );
        // Mat duckImage = processFrame( input, "duck" );
        Mat customImage = processFrame( input, "custom");
        double eleValue = Core.sumElems( elementImage ).val[0] / (elementImage.rows( ) * elementImage.cols( )) / 255;
        // double duckValue = Core.sumElems( duckImage ).val[0] / (duckImage.rows( ) * duckImage.cols( )) / 255;
        double customValue = Core.sumElems( customImage ).val[0] / (customImage.rows( ) * customImage.cols( )) / 255;
        // telemetry.update( );
        if( eleValue < customValue )
            return customImage;
        return elementImage;
    } public BarcodePosition getBarcodePosition() {
        return barcodePosition;
    }}
} //End Brace