package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "ACJA02")
@Disabled
public class ACJA02 extends LinearOpMode {

    private DcMotor flMotor;
    private DcMotor frMotor;
    private DcMotor blMotor;
    private DcMotor brMotor;
    private DcMotor liftMotor;
    //private DcMotor intakeMotor;
    private DcMotor tiltMotor;

    //Convert from the counts per revolution of the encoder to counts per inch
    static final double HD_COUNTS_PER_REV = 28;
    static final double DRIVE_GEAR_REDUCTION = 20.15293;
    static final double WHEEL_CIRCUMFERENCE_MM = 90 * Math.PI;
    static final double DRIVE_COUNTS_PER_MM = (HD_COUNTS_PER_REV * DRIVE_GEAR_REDUCTION) / WHEEL_CIRCUMFERENCE_MM;
    static final double DRIVE_COUNTS_PER_IN = DRIVE_COUNTS_PER_MM * 25.4;

    //Create elapsed time variable and an instance of elapsed time
    private ElapsedTime runtime = new ElapsedTime();

    // Drive function with 3 parameters
    private void drive(double power, double leftInches, double rightInches) {
        int flTarget;
        int frTarget;
        int blTarget;
        int brTarget;

        if (opModeIsActive()) {
            // Create target positions
            flTarget = flMotor.getCurrentPosition() + (int)(leftInches * DRIVE_COUNTS_PER_IN);
            frTarget = frMotor.getCurrentPosition() + (int)(rightInches * DRIVE_COUNTS_PER_IN);
            blTarget = blMotor.getCurrentPosition() + (int)(leftInches * DRIVE_COUNTS_PER_IN);
            brTarget = brMotor.getCurrentPosition() + (int)(rightInches * DRIVE_COUNTS_PER_IN);

            // set target position
            flMotor.setTargetPosition(flTarget);
            frMotor.setTargetPosition(frTarget);
            blMotor.setTargetPosition(blTarget);
            brMotor.setTargetPosition(brTarget);

            //switch to run to position mode
            flMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            blMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            brMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //run to position at the designated power
            flMotor.setPower(power);
            frMotor.setPower(power);
            blMotor.setPower(power);
            brMotor.setPower(power);

            // wait until both motors are no longer busy running to position
            while (opModeIsActive() && (flMotor.isBusy() || frMotor.isBusy() || blMotor.isBusy() || brMotor.isBusy())) {
            }

            // set motor power back to 0
            blMotor.setPower(0);
            brMotor.setPower(0);
            flMotor.setPower(0);
            frMotor.setPower(0);
        }
    }


    @Override
    public void runOpMode() {

        flMotor = hardwareMap.get(DcMotor.class, "flMotor");
        frMotor = hardwareMap.get(DcMotor.class, "frMotor");
        blMotor = hardwareMap.get(DcMotor.class, "blMotor");
        brMotor = hardwareMap.get(DcMotor.class, "brMotor");
        liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");
        tiltMotor = hardwareMap.get(DcMotor.class, "tiltMotor");


        // flMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        if (opModeIsActive()) {

            //segment 1
            drive(0.7, 30, 15);

            runtime.reset(); // reset elapsed time timer

//            //segment 2 - lift arm, drive to shipping hub, outtake freight
//            while (opModeIsActive() && runtime.seconds() <= 7) {
//
//                //lift arm and hold
//                Arm.setTargetPosition(120);
//                Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                Arm.setPower(0.3);
//
//                //drive forward for 1 second
//                while (runtime.seconds() > 2 && runtime.seconds() <= 3) {
//                    drive(0.4, 4, 4);
//                }
//
//                //run intake
//                while (runtime.seconds() > 4 && runtime.seconds() <= 7) {
//                    Intake.setPower(-0.6);
//                }
//
//                // turn off arm and intake
//                Arm.setPower(0);
//                Intake.setPower(0);
//
//                //segment 3 - reverse to get better angle
//                drive(0.7, -15, -30);
//
//                //segment 4 - drive into warehouse
//                drive(1, 90, 90);
            }
        }
    }