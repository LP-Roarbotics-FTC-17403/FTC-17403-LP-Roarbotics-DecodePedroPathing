package org.firstinspires.ftc.teamcode.TroubleShooting;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.SubSystems.CameraSystem;
import org.firstinspires.ftc.teamcode.SubSystems.LED;

/*
 * This file includes a teleop (driver-controlled) file for the goBILDA® Robot in 3 Days for the
 * 2025-2026 FIRST® Tech Challenge season DECODE™!
 */

@TeleOp(group = "TroubleShoot")
//@Disabled
public class AprilTagPoseDetection extends LinearOpMode {
    final double FEED_TIME_SECONDS = 2.80; //The feeder servos run this long when a shot is requested.
    final double STOP_SPEED = 0.0; //We send this power to the servos when we want them to stop.
    final double FULL_SPEED = 1.0;

    final double LAUNCHER_CLOSE_TARGET_VELOCITY = 1200; //in ticks/second for the close goal.
    final double LAUNCHER_CLOSE_MIN_VELOCITY = 1175; //minimum required to start a shot for close goal.

    final double LAUNCHER_FAR_TARGET_VELOCITY = 1400; //Target velocity for far goal
    final double LAUNCHER_FAR_MIN_VELOCITY = 1375; //minimum required to start a shot for far goal.

    double launcherTarget = LAUNCHER_CLOSE_TARGET_VELOCITY; //These variables allow
    double launcherMin = LAUNCHER_CLOSE_MIN_VELOCITY;

    final double LEFT_POSITION = 0.3162; //the left and right position for the diverter servo
    final double RIGHT_POSITION = 0.035;

    // Declare OpMode members.
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotorEx leftLauncher = null;
    private DcMotorEx rightLauncher = null;
    private DcMotor intake = null;
    private DcMotor transfer = null;
    private CRServo leftFeeder = null;
    private CRServo rightFeeder = null;
    private Servo diverter = null;

    private LED leftLight;
    private LED rightLight;
    private CameraSystem camera;

    ElapsedTime leftFeederTimer = new ElapsedTime();
    ElapsedTime rightFeederTimer = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {

        leftLight = new LED(hardwareMap, "left_light");
        rightLight = new LED(hardwareMap, "right_light");
        camera = new CameraSystem(hardwareMap);

        camera.cameraOn();
        waitForStart();

        while(opModeIsActive()){
            camera.updateDesiredDetection(24);

            double currentYaw = camera.getYaw();
            double currentPitch = camera.getPitch();
            double currentRoll = camera.getPitch();
            double currentX = camera.getX();
            double currentY = camera.getY();
            double currentZ = camera.getZ();
            telemetry.addData("current Yaw: ", currentYaw);
            telemetry.addData("current Pitch: ", currentPitch);
            telemetry.addData("current Roll: ", currentRoll);
            telemetry.addData("current X: ", currentX);
            telemetry.addData("current Y: ", currentY);
            telemetry.addData("current Z: ", currentZ);
            telemetry.update();

            double angleInDegrees = getAngle(currentX, currentZ);
            if(camera.getDetected()){
                if(Math.abs(angleInDegrees) < 3){
                    rightLight.setGreen();
                    leftLight.setGreen();
                }else{
                    if(angleInDegrees>3){
                        rightLight.setPurple();
                        leftLight.setRed();
                    }else if(angleInDegrees<-3){
                        rightLight.setRed();
                        leftLight.setPurple();
                    }
                }
            }else{
                rightLight.setRed();
                leftLight.setRed();
            }


        }

    }

    public double getAngle(double x, double z){
        return Math.toDegrees(Math.atan2(x, z));
    }




}