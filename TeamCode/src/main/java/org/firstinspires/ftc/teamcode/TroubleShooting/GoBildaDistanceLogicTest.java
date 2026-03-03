package org.firstinspires.ftc.teamcode.TroubleShooting;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Logics.GoBildaDistanceSensorLogic;
import org.firstinspires.ftc.teamcode.SubSystems.Hammer;
import org.firstinspires.ftc.teamcode.SubSystems.Inhaler;

/*
 *
 */

@TeleOp(group = "TroubleShoot")
//@Disabled
public class GoBildaDistanceLogicTest extends LinearOpMode {

    private Inhaler inhaler;
    private Inhaler transfer;

    private Hammer diverter;
    private Servo rightGate;
    private Servo leftGate;

    private boolean divertOn = false;
    final double LEFT_OPEN_POSITION = 0;
    final double LEFT_CLOSE_POSITION = 0.225;

    final double RIGHT_CLOSE_POSITION = 0;
    final double RIGHT_OPEN_POSITION = 0.225;
    //change this to change the amount of time waiting to proceed to shooting
    private double timeWaiting = 9999;
    private int state = 0;

    // Declare OpMode members.
    private GoBildaDistanceSensorLogic detection = new GoBildaDistanceSensorLogic();

    private ElapsedTime timer = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {
        inhaler = new Inhaler(hardwareMap, "intake");
        transfer = new Inhaler(hardwareMap, "transfer");
        leftGate = hardwareMap.get(Servo.class, "left_gate");
        rightGate = hardwareMap.get(Servo.class, "right_gate");
        inhaler.initialize(true);
        transfer.initialize(true);
        detection.init(hardwareMap);
        waitForStart();

        while(opModeIsActive()) {
            //continuous update to sensor logic
            detection.update(divertOn);

            //controls the process of intaking
            if(state == 0){
                inhaler.inhale_on();
                transfer.inhale_on();
            }else if(state == 1){
                detection.startDetect();
                timer.reset();
                state = 2;
            }else if(state == 2){
                if(detection.loaded() || timer.seconds() > timeWaiting){
                    inhaler.inhale_off();
                    transfer.inhale_off();
                    telemetry.addLine("Full storage!");
                }
            }
            if(gamepad1.x){
                rightGate.setPosition(RIGHT_OPEN_POSITION);
                leftGate.setPosition(LEFT_OPEN_POSITION);
            }
            if(gamepad1.y){
                rightGate.setPosition(RIGHT_CLOSE_POSITION);
                leftGate.setPosition(LEFT_CLOSE_POSITION);
            }
            if(gamepad1.b){
                divertOn = !divertOn;
            }

            //manually controls and telemetry displays
            if(gamepad1.a){
                state = 1;
            }
            telemetry.addData("Press 'b' to switch diverter. Current Diverter: ", divertOn);
            telemetry.addData("Press 'a' on gamepad 1 to move to next state, \n current state: ", state);
            if(state == 2) {
                telemetry.addData("loaded? : ", detection.loaded());
            }else{
                telemetry.addLine("Press 'b' to start over.");
            }
            telemetry.addLine("Current sensor behavior: " + detection.currentMessage());
            telemetry.addData("Left Distance: ", detection.getDistance());
            telemetry.update();
        }
    }





}