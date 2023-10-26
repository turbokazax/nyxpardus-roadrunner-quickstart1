package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@TeleOp
public class PositionControllingPIDTeleOP extends LinearOpMode {
    // constanti PIDa mojno huyarit' public static
    double kp = 0.008, ki = 0.0, kd = 0.0005;
//    norm koefi: kp=0.008, ki=0.0, kd=0.0004 (problem: power <1)

    DcMotorEx motorLeft, motorRight;
    ElapsedTime timer;
//    PIDFController kakoytopid;

    @Override
    public void runOpMode()
    {
//        kakoytopid = new PIDFController(kp, ki, kd, 0);
        timer = new ElapsedTime();

        // init
        motorLeft = hardwareMap.get(DcMotorEx.class, "motor1");
        motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double accum = 0;
        double prevError = 0;
        double coef = 3000.0;
        double target = 0;
        double prevTime = 0;
        waitForStart();
        double prevPos = 0;
        while (opModeIsActive())
        {
//            if(gamepad1.a) motorLeft.setPower(0.0);

            double elapsedTime = timer.milliseconds() / 1000.0;
            timer.reset();
            prevTime = elapsedTime;

//            PIDR - потенциальный интегральный дифференцальный регулятор
            target += gamepad1.left_stick_x * 2400 * elapsedTime;
//            coef+=elapsedTime*100;
            double currentPosition = motorLeft.getCurrentPosition();
            prevPos = currentPosition;
//            double powerGotovy = kakoytopid.calculate(currentPosition, target);//

            double error = target - currentPosition;

//            P
            double p = error;

//            I
            accum += error;
            double i = accum;

//            D
            double d = (error - prevError) / elapsedTime;
            prevError = error;
//
            double power = p * kp + i * ki + d * kd;
            telemetry.addData("power", power);
            telemetry.addData("target", target);
            telemetry.addData("current",currentPosition );
//            double prevPos = currentPosition;
//            double prevTime = elapsedTime;

            telemetry.addData("ROC", (currentPosition-prevPos)/(elapsedTime - prevTime));
            telemetry.update();
//            double power = powerGotovy;
            motorLeft.setPower(power);
        }
        // loop
    }
}

