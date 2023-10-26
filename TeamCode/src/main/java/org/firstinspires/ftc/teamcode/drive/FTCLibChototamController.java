package org.firstinspires.ftc.teamcode.drive;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class FTCLibChototamController extends LinearOpMode {
    double kp = 0.001, ki = 0, kd = 0;

    Motor motorLeft;
    ElapsedTime timer;
    PIDFController kakoytopid;
    double currentInputPosition = 0;
    @Override
    public void runOpMode()
    {
        kakoytopid = new PIDFController(kp, ki, kd, 0);
        timer = new ElapsedTime();

        // init
        motorLeft = new Motor(hardwareMap, "motor1");
        motorLeft.setRunMode(Motor.RunMode.VelocityControl);

        double accum = 0;
        double prevError = 0;

        while (opModeIsActive())
        {
            double elapsedTime = timer.milliseconds() / 1000.0;
            currentInputPosition += gamepad1.right_stick_x * elapsedTime;

//            PIDR - потенциальный интегральный дифференцальный регулятор
//            double target = gamepad1.left_stick_x * 2240;
            double target = currentInputPosition;
            double currentPosition = motorLeft.getCurrentPosition();

            double powerGotovy = kakoytopid.calculate(currentPosition, target);//
            // dlya lifta ( bez velocity control ):
            double powerLift = kakoytopid.calculate(currentPosition, target) + 0.04; //mojno poschitat (ya ne fizik)))


//
//            double error = target - currentPosition;
//
////            P
//            double p = error;
//
////            I
//            accum += error;
//            double i = accum;
//
////            D
//            double d = (error - prevError) / elapsedTime;
//            prevError = error;
//
////            double power = p * kp + i * ki + d * kd;
            double power = powerGotovy;
            motorLeft.set(power);
        }
        // loop
    }
}
