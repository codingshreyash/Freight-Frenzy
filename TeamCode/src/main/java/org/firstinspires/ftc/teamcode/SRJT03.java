package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
@TeleOp(name = "SRJT03", group = "Opmode")
@Disabled
public class SRJT03 extends OpMode {
    private DcMotor tiltMotor;
    liftPIDController liftPIDController;
    @Override

    public void init() {
        liftPIDController = new liftPIDController(0, 0, 0);
        tiltMotor = hardwareMap.get(DcMotor.class, "tiltMotor");
    }
    public void start() {
        tiltMotor.setPower(0);
    }
    public void loop() {
        intakeLift();
    }
    public void stop() {
        tiltMotor.setPower(0);
    }
    public void tiltToPosition(double tiltTargetPosition, double movementSpeed) {
        double startTime = 0;
        double stopState = 0;
        while (stopState < 500) {
            double power = liftPIDController.getOutput(tiltMotor.getCurrentPosition(), tiltTargetPosition);
            tiltMotor.setPower(power);
            if (Math.abs(liftPIDController.getError()) < 2) {
                stopState = System.currentTimeMillis() - startTime;
            } else {
                startTime = System.currentTimeMillis();
            }
        }
    }
    public void intakeLift() {
        tiltMotor.setDirection(DcMotor.Direction.FORWARD);
        double tiltPower = -.80;
        tiltMotor.setPower(gamepad2.right_stick_y * tiltPower); //up
        telemetry.addData("tilt motor current postion", tiltMotor.getCurrentPosition());



    } //End Brace
}