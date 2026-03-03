package org.firstinspires.ftc.teamcode.TroubleShooting;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

/*
 * Credits to Brogan Matts
 */

@TeleOp(group = "TroubleShoot")
//@Disabled
public class FlyWheelTuning extends OpMode {
    //keep motor and corresponding motor hardwareMap name consistent in order.
    private DcMotorEx flywheel1;
    private DcMotorEx flywheel2;

    private String flywheelName1 = "left_launcher";
    private String flywheelName2 = "right_launcher";

    //Set your desired target velocity, should be the velocity you are using in the game.
    private double velocity1 =1140;
    private double velocity2 = 1200;
    private double velocity3 = 1400;

    private double targetHighVelocity = 0;

    //don't change this, used for proportion value tuning
    private double testingVelocity = 100;

    private double currentTargetVelocity = targetHighVelocity;

    private double[] stepSizes = {10.0, 1.0, 0.1, 0.01, 0.001, 0.0001};

    private double p = 0;
    private double f = 0;


    private int stepIndex = 1;
    @Override
    public void init(){

          flywheel1 = hardwareMap.get(DcMotorEx.class, flywheelName1);
          flywheel1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
          //flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
          flywheel1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(p, 0, 0 ,f));
        flywheel2 = hardwareMap.get(DcMotorEx.class, flywheelName2);
        flywheel2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel2.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheel2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(p, 0, 0 ,f));
          telemetry.addLine("Init Complete!");

    }
    @Override
    public void loop(){
        if(gamepad1.yWasPressed()){
            if(currentTargetVelocity ==targetHighVelocity){
                currentTargetVelocity = testingVelocity;
            }else{
                currentTargetVelocity = targetHighVelocity;
            }
        }

        if(gamepad1.bWasPressed()){
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

        if(gamepad1.dpadLeftWasPressed()){
            f -=stepSizes[stepIndex];
        }

        if(gamepad1.dpadRightWasPressed()){
            f += stepSizes[stepIndex];
        }

        if(gamepad1.dpadUpWasPressed()){
            p += stepSizes[stepIndex];
        }

        if(gamepad1.dpadDownWasPressed()){
            p -= stepSizes[stepIndex];
        }

        if(gamepad2.a){
            targetHighVelocity = velocity1;
            currentTargetVelocity = velocity1;
        }
        if(gamepad2.b){
            targetHighVelocity = velocity2;
            currentTargetVelocity = velocity2;
        }
        if(gamepad2.x){
            targetHighVelocity = velocity3;
            currentTargetVelocity = velocity3;
        }


            flywheel1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(p, 0, 0, f));
            flywheel1.setVelocity(currentTargetVelocity);
        flywheel2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(p, 0, 0, f));
        flywheel2.setVelocity(currentTargetVelocity);


            telemetry.addData("target velocity: ", currentTargetVelocity);
            telemetry.addData(  "flywheel velocity: ",flywheel1.getVelocity());
        telemetry.addData(  "flywheel velocity: ",flywheel2.getVelocity());

        telemetry.addLine("--------------------------------");
        telemetry.addData("p value(dpad up/down): ", p);
        telemetry.addData("f value(dpad right/left): ", f);
        telemetry.addData("current step size: ", stepSizes[stepIndex]);
        telemetry.update();

    }


}