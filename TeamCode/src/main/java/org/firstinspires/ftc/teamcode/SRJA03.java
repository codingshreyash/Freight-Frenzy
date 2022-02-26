package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Function;

import org.opencv.core.Core;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Point;

@Autonomous(name = "SRJA03", group = "")
public class SRJA03 extends OpMode {
    public static ArrayList<Function<Void, Boolean>> autoSequence = new ArrayList<>();
    public int sequenceCount;

    /*
     * Motors:
     * CH0: intakeMotor
     * CH1: frMotor
     * CH2: brMotor
     * CH3: liftMotor
     *
     * EH0: flMotor
     * EH1: blMotor
     * EH2:
     * EH3: tiltMotor
     */
    private DcMotor flMotor;
    private DcMotor frMotor;
    private DcMotor blMotor;
    private DcMotor brMotor;
    private DcMotor liftMotor;
    private DcMotor intakeMotor;
    private DcMotor tiltMotor;
    private Servo tiltServo;
    private Servo dropServo;
    private CRServo carouselRight;
    private CRServo carouselLeft;

    /*
     * Servos:
     *
     */
    private ElapsedTime runtime = new ElapsedTime();
    private BNO055IMU imu;
    Orientation threeAngles;
    double lastAngle;
    double globalAngle;
    boolean droveAlready;
    boolean rotatedAlready;
    Orientation lastAngles = new Orientation();
    private BarcodePosition barcodePosition = BarcodePosition.NOT_FOUND;

    public static enum BarcodePosition {
        LEFT {
            @Override
            public String toString() {
                return "Left jit";
            }
        },
        MIDDLE {
            @Override
            public String toString() {
                return "Middle shlatt";
            }
        },
        RIGHT {
            @Override
            public String toString() {
                return "Right cuh";
            }
        },
        NOT_FOUND {
            @Override
            public String toString() {
                return "Not found slime";
            }
        }
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


    @Override
    public void init() {
        //Important
        autoSequence.clear();
        autoSequence.add((Void) -> detectImage());
        sequenceCount = 0;
        flMotor = hardwareMap.dcMotor.get("flMotor");
        frMotor = hardwareMap.dcMotor.get("frMotor");
        blMotor = hardwareMap.dcMotor.get("blMotor");
        brMotor = hardwareMap.dcMotor.get("brMotor");
        intakeMotor = hardwareMap.dcMotor.get("intakeMotor");
        liftMotor = hardwareMap.get(DcMotorEx.class, "liftMotor");
        tiltMotor = hardwareMap.get(DcMotorEx.class, "tiltMotor");
        tiltServo = hardwareMap.get(Servo.class, "tiltServo");
        dropServo = hardwareMap.get(Servo.class, "dropServo");
        carouselRight = hardwareMap.get(CRServo.class, "carouselRight");
        carouselLeft = hardwareMap.get(CRServo.class, "carouselLeft");

        //Not Important
        imu = hardwareMap.get(BNO055IMU.class, "imu 1");
        threeAngles = new Orientation();
        droveAlready = false;
        rotatedAlready = false;
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(parameters);

        //Important again
        flMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        blMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        brMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    @Override
    public void loop() {
        if (sequenceCount < autoSequence.size()) {
            Function<Void, Boolean> currentFunction = autoSequence.get(sequenceCount); // get the function from autoSequence at index sequenceCount
            boolean currentFunctionState = false; // run the current function and get its state (true or false)
            currentFunctionState = currentFunction.apply(null);
            if (currentFunctionState) {  // if the state is true, it means the current function has completed running
                sequenceCount++; // increment sequenceCount to begin running the next function in the next iteration
                resetSensors();
            }
        }
        logTelemetry();
    }

    public void resetSensors() {
        flMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        blMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        brMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        tiltMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        resetAngle();
    }

    private void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        globalAngle = 0;

    }

    private double getAngle() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES); //
        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;
        if (deltaAngle < -180) {
            deltaAngle += 360;
        } else if (deltaAngle > 180) {
            deltaAngle -= 360;
        }
        globalAngle += deltaAngle;
        lastAngles = angles;
        return globalAngle;
    }

//    private boolean driveDistance(double distance, double drivePower) {
//        // set motors to run forward for 2500 encoder counts.
//        double inchesToTicks = 40.584;
//        drivePower /= 100;
//        int ticks = (int) (distance * inchesToTicks);
//
//        flMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        blMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        frMotor.setDirection(DcMotorSimple.Direction.FORWARD);
//        brMotor.setDirection(DcMotorSimple.Direction.FORWARD);
//
//        flMotor.setTargetPosition(ticks);
//        frMotor.setTargetPosition(ticks);
//        blMotor.setTargetPosition(ticks);
//        brMotor.setTargetPosition(ticks);
//
//        flMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        frMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        blMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        brMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        if (Math.abs(flMotor.getCurrentPosition()) < flMotor.getTargetPosition()) {
//            flMotor.setPower(drivePower);
//            frMotor.setPower(drivePower);
//            blMotor.setPower(drivePower);
//            brMotor.setPower(drivePower);
//            return false;
//        } else {
//            frMotor.setPower(0);
//            flMotor.setPower(0);
//            brMotor.setPower(0);
//            blMotor.setPower(0);
//            //droveAlready = true;
//            return true;
//        }
//    }

    private boolean driveDistanceBackwards(double distance, double drivePower) {
        // set motors to run forward for 2500 encoder counts.
        double inchesToTicks = 40.584;
        drivePower /= 100;

        int ticks = (int) (distance * inchesToTicks);

        flMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        blMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        brMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        flMotor.setTargetPosition(ticks);
        frMotor.setTargetPosition(ticks);
        blMotor.setTargetPosition(ticks);
        brMotor.setTargetPosition(ticks);

        flMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        blMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        brMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if (Math.abs(flMotor.getCurrentPosition()) < flMotor.getTargetPosition()) {
            flMotor.setPower(drivePower);
            frMotor.setPower(drivePower);
            blMotor.setPower(drivePower);
            brMotor.setPower(drivePower);
            return false;
        } else {
            frMotor.setPower(0);
            flMotor.setPower(0);
            brMotor.setPower(0);
            blMotor.setPower(0);
            //droveAlready = true;
            return true;
        }
    }

    private boolean rotateLeft(double time, double rotatePower) {
        rotatePower /= 100;
        flMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        blMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        brMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        blMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        brMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        runtime.reset();

        while (runtime.milliseconds() <= time) {
            telemetry.addData("current time", runtime.milliseconds());
            flMotor.setPower(rotatePower);
            blMotor.setPower(rotatePower);
            frMotor.setPower(-rotatePower);
            brMotor.setPower(-rotatePower);
        }
        flMotor.setPower(0);
        blMotor.setPower(0);
        frMotor.setPower(0);
        brMotor.setPower(0);
        return true;
    }

    private boolean rotateRight(double time, double rotatePower) {
        rotatePower /= 100;
        flMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        blMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        brMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frMotor.setDirection(DcMotorSimple.Direction.FORWARD); //Omni wheel
        brMotor.setDirection(DcMotorSimple.Direction.FORWARD); // not using
        flMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        blMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        runtime.reset();

        while (runtime.milliseconds() <= time) {
            telemetry.addData("current time", runtime.milliseconds());
            flMotor.setPower(rotatePower);
            blMotor.setPower(rotatePower);
            frMotor.setPower(-rotatePower);
            brMotor.setPower(-rotatePower);
        }
        flMotor.setPower(0);
        blMotor.setPower(0);
        frMotor.setPower(0);
        brMotor.setPower(0);

        return true;
    }

    private boolean strafeRight(double time, double strafePower) {
        strafePower /= 100;
        flMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        blMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        brMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frMotor.setDirection(DcMotorSimple.Direction.FORWARD); //Omni wheel
        brMotor.setDirection(DcMotorSimple.Direction.FORWARD); // not using
        flMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        blMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        runtime.reset();

        while (runtime.milliseconds() <= time) {
            telemetry.addData("current time", runtime.milliseconds());
            flMotor.setPower(strafePower);
            blMotor.setPower(-strafePower);
            frMotor.setPower(-strafePower);
            brMotor.setPower(strafePower);
        }
        flMotor.setPower(0);
        blMotor.setPower(0);
        frMotor.setPower(0);
        brMotor.setPower(0);

        return true;
    }

    private boolean strafeLeft(double time, double strafePower) {
        strafePower /= 100;
        flMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        blMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        brMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frMotor.setDirection(DcMotorSimple.Direction.FORWARD); //Omni wheel
        brMotor.setDirection(DcMotorSimple.Direction.FORWARD); // not using
        flMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        blMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        runtime.reset();

        while (runtime.milliseconds() <= time) {
            telemetry.addData("current time", runtime.milliseconds());
            flMotor.setPower(-strafePower);
            blMotor.setPower(strafePower);
            frMotor.setPower(strafePower);
            brMotor.setPower(-strafePower);
        }
        flMotor.setPower(0);
        blMotor.setPower(0);
        frMotor.setPower(0);
        brMotor.setPower(0);

        return true;
    }

    private boolean moveForward(double time, double strafePower) {
        strafePower /= 100;
        flMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        blMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        brMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frMotor.setDirection(DcMotorSimple.Direction.FORWARD); //Omni wheel
        brMotor.setDirection(DcMotorSimple.Direction.FORWARD); // not using
        flMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        blMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        runtime.reset();

        while (runtime.milliseconds() <= time) {
            telemetry.addData("current time", runtime.milliseconds());
            flMotor.setPower(strafePower);
            blMotor.setPower(strafePower);
            frMotor.setPower(strafePower);
            brMotor.setPower(strafePower);
        }
        if (runtime.milliseconds() <= time)
            return false;
        flMotor.setPower(0);
        blMotor.setPower(0);
        frMotor.setPower(0);
        brMotor.setPower(0);

        return true;
    }

    public boolean xCommand() {
        dropServo.setPosition(1);
        liftMotor.setTargetPosition(-7);
        liftMotor.setPower(0.4);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        return true;
    }


    public boolean bCommand() {
        dropServo.setPosition(1);
        liftMotor.setTargetPosition(-7);
        liftMotor.setPower(0.4);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        return true;
    }

    public boolean aCommand() {
        tiltMotor.setTargetPosition(-416);
        tiltMotor.setPower(.7);
        tiltMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

//        liftMotor.setTargetPosition(-7);
//        liftMotor.setPower(0.6);
//        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        tiltServo.setPosition(.25);
        dropServo.setPosition(1);

        if (tiltMotor.getCurrentPosition() >= -216) {
            return false;
        }
        return true;
    }

    public boolean spinWheelBlue() {
        if (intakeMotor.getCurrentPosition() <= -1603) {
            intakeMotor.setTargetPosition(-1603);
            intakeMotor.setPower(.7);
            intakeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            carouselLeft.setPower(1);
            return false;
        }
        carouselLeft.setPower(0);
        return true;
    }

    public boolean yCommand() {
//        tiltMotor.setTargetPosition(-479);
//        tiltMotor.setPower(.7);
//        tiltMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        liftMotor.setTargetPosition(1247);
        liftMotor.setPower(0.40);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        tiltServo.setPosition(.30);
        dropServo.setPosition(1);
        return true;
    }




    private boolean intake(double distance, double intakePower) {
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        double inchesToTicks = 40.584;
        intakePower /= 100;
        int ticks = (int) (distance * inchesToTicks);
        intakeMotor.setTargetPosition(ticks);
        intakeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if (Math.abs(intakeMotor.getCurrentPosition()) < intakeMotor.getTargetPosition()) {
            intakeMotor.setPower(intakePower);
            intakeMotor.setPower(intakePower);
            return false;
        } else {
            intakeMotor.setPower(0);
            intakeMotor.setPower(0);
            return true;
        }
    }


    private Boolean detectImage() {
        DuckDetectionPipeline vision = new DuckDetectionPipeline();
        barcodePosition = vision.getBarcodePosition();

        if (barcodePosition.equals(BarcodePosition.LEFT)) {

            autoSequence.add((Void) -> rotateRight(2500, 85)); // 2500 -> 1.75 turns
            //85 percent speed : 1000ms - 70 inch
            autoSequence.add((Void) -> strafeLeft(300, 50)); //strafe away from wall
            autoSequence.add((Void) -> moveForward(700, -50)); //move forward to cara
            autoSequence.add((Void) -> moveForward(660, 50)); //move backward from cara
            autoSequence.add((Void) -> rotateRight(500, 85)); //180 rotate right to face tower\
            autoSequence.add((Void) -> rotateRight(178, 85)); //180 rotate right to face tower
            autoSequence.add((Void) -> moveForward(600, -85)); //move forward to cara

            return true;

        } else if (barcodePosition.equals(BarcodePosition.MIDDLE)) {
            autoSequence.add((Void) -> rotateRight(2500, 85));
            // 2500 -> 1.75 turns
            //85 percent speed : 1000ms - 70 inch
            autoSequence.add((Void) -> strafeLeft(300, 50)); //strafe away from wall
            autoSequence.add((Void) -> moveForward(700, -50)); //move forward to cara
            autoSequence.add((Void) -> moveForward(660, 50)); //move backward from cara
            autoSequence.add((Void) -> rotateRight(500, 85)); //180 rotate right to face tower\
            autoSequence.add((Void) -> rotateRight(178, 85)); //180 rotate right to face tower
            autoSequence.add((Void) -> moveForward(600, -85)); //move forward to cara

            return true;

        } else if (barcodePosition.equals(BarcodePosition.RIGHT)) {
            autoSequence.add((Void) -> rotateRight(2500, 85)); // 2500 -> 1.75 turns
            //85 percent speed : 1000ms - 70 inch
            autoSequence.add((Void) -> strafeLeft(300, 50)); //strafe away from wall
            autoSequence.add((Void) -> moveForward(700, -50)); //move forward to cara
            autoSequence.add((Void) -> moveForward(660, 50)); //move backward from cara
            autoSequence.add((Void) -> rotateRight(500, 85)); //180 rotate right to face tower\
            autoSequence.add((Void) -> rotateRight(178, 85)); //180 rotate right to face tower
            autoSequence.add((Void) -> moveForward(600, -85)); //move forward to cara
            return true;

        }



        /*
        Detect Team shippinging elements
        Spin the carosoul
        Deliver to correct level
        Pick up 1 freight and deliver
        Park all the way warehouse
         */
        //here

        else if (barcodePosition.equals(BarcodePosition.NOT_FOUND)) {
            carouselLeft.setPower(1);



            telemetry.addData("not found", 1);
            // 85 percent speed : 1000ms - 70 inch

            autoSequence.add((Void) -> moveForward(150, 85));
            autoSequence.add((Void) -> spinWheelBlue());

            //carouselRight.setPower(0);

            autoSequence.add((Void) -> moveForward(153, -85));
            autoSequence.add((Void) -> strafeRight(50, 85));
            autoSequence.add((Void) -> rotateRight(1000, 85));
            autoSequence.add((Void) -> yCommand());

//            autoSequence.add((Void) -> moveForward(150, 85));
//            autoSequence.add((Void) -> moveForward(150, 85));

//            autoSequence.add((Void) -> rotateRight(2500, 85)); // 2500 -> 1.75 turns
//            autoSequence.add((Void) -> strafeLeft(300, 50)); //strafe away from wall
//            autoSequence.add((Void) -> moveForward(700, -50)); //move forward to cara
//            autoSequence.add((Void) -> moveForward(660, 50)); //move backward from cara
//            autoSequence.add((Void) -> rotateRight(500, 85)); //180 rotate right to face tower\
//            autoSequence.add((Void) -> rotateRight(178, 85)); //180 rotate right to face tower
//            autoSequence.add((Void) -> moveForward(600, -85)); //move forward to cara
            // autoSequence.add((Void) -> deliverFreight(0.8, 0.8));
            return true;
        }
        return true;
    }


    public class DuckDetectionPipeline extends OpenCvPipeline
    {

        public Mat processFrame(Mat input, String type) {
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
        }

        @Override
        public Mat processFrame(Mat input) {
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
        }

        public BarcodePosition getBarcodePosition() {
            return barcodePosition;
        }
    }

    public void logTelemetry() {
        telemetry.addData("Barcode Position: ", barcodePosition.toString());
        telemetry.update();
    }

}

