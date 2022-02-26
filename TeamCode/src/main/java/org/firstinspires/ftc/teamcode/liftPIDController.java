package org.firstinspires.ftc.teamcode;

public class liftPIDController {
        private double KP;
        private double KI;
        private double KD;
        private double previousTime = 0;
        private double error;
        private double i = 0;
        private double d = 0;
        private double deltaTime = 0;
        private double previousError = 0;

        public liftPIDController(double KP , double KI , double KD){
            this.KP = KP;
            this.KI = KI;
            this.KD = KD;
        }

        public double getOutput(double currPos, double targetPos){
            this.error = targetPos - currPos;
            this.previousError = error;
            double P = KP * error; // Proportional term : KP constant * the error of the system
            this.deltaTime = System.currentTimeMillis() - previousTime;
            this.previousTime = System.currentTimeMillis();
            this.i += currPos > targetPos * 0.8 ? deltaTime * error : 0;
            double I = KI * i;
            this.d = (error - previousError) / deltaTime;
            double D = KD * d;

            return P + I + D;
        }

        public double getError(){
            return this.error;
        }

        public void setConstants(double KP , double KI , double KD){
            this.KP = KP;
            this.KI = KI;
            this.KD = KD;
        }
    }

