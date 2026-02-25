package org.firstinspires.ftc.teamcode.TroubleShooting;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Auto.RevV3ColorSensorLogic;
import org.firstinspires.ftc.teamcode.SubSystems.CameraSystem;
import org.firstinspires.ftc.teamcode.SubSystems.Inhaler;
import org.firstinspires.ftc.teamcode.SubSystems.LED;

/*
 * This file includes a teleop (driver-controlled) file for the goBILDA® Robot in 3 Days for the
 * 2025-2026 FIRST® Tech Challenge season DECODE™!
 */

@TeleOp(group = "TroubleShoot")
//@Disabled
public class RevColorSensorLogicTest extends LinearOpMode {
    final double FEED_TIME_SECONDS = 2.80; //The feeder servos run this long when a shot is requested.
    final double STOP_SPEED = 0.0; //We send this power to the servos when we want them to stop.
    final double FULL_SPEED = 1.0;
    private Inhaler inhaler;
    private Inhaler transfer;

    final double LAUNCHER_CLOSE_TARGET_VELOCITY = 1200; //in ticks/second for the close goal.
    final double LAUNCHER_CLOSE_MIN_VELOCITY = 1175; //minimum required to start a shot for close goal.

    final double LAUNCHER_FAR_TARGET_VELOCITY = 1400; //Target velocity for far goal
    final double LAUNCHER_FAR_MIN_VELOCITY = 1375; //minimum required to start a shot for far goal.

    double launcherTarget = LAUNCHER_CLOSE_TARGET_VELOCITY; //These variables allow
    double launcherMin = LAUNCHER_CLOSE_MIN_VELOCITY;

    final double LEFT_POSITION = 0.3162; //the left and right position for the diverter servo
    final double RIGHT_POSITION = 0.035;

    private int state = 0;

    // Declare OpMode members.
    private RevV3ColorSensorLogic detection = new RevV3ColorSensorLogic();

    ElapsedTime leftFeederTimer = new ElapsedTime();
    ElapsedTime rightFeederTimer = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {
        inhaler = new Inhaler(hardwareMap, "intake");
        transfer = new Inhaler(hardwareMap, "transfer");
        detection.init(hardwareMap);
        waitForStart();

        while(opModeIsActive()) {
            detection.update();
            if(state == 0){
                inhaler.inhale_on();
                transfer.inhale_on();
            }else if(state == 1){
                detection.startDetect();
                state = 2;
            }else if(state == 2){
                if(detection.loaded()){
                    inhaler.inhale_off();
                    transfer.inhale_off();
                }
            }
            if(gamepad1.a){
                state++;
            }
            telemetry.addData("Press 'a' on gamepad 1 to move to next state, \n current state: ", state);
            telemetry.addData("loaded?(considered true if not detecting) : ", detection.loaded());
            telemetry.update();
        }
    }





}