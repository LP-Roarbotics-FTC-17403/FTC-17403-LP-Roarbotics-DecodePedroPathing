package org.firstinspires.ftc.teamcode.Auto;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.SubSystems.Firecracker;
import org.firstinspires.ftc.teamcode.SubSystems.Inhaler;

public class LauncherStateMachine {
    private DcMotorEx rightLauncher = null;
    private DcMotorEx leftLauncher = null;

    private DcMotorEx transfer = null;
    private CRServo leftFeeder = null;
    private CRServo rightFeeder = null;

    private double launchTime = 0.8;

    private double gapBetweenLaunch = 0;
    private double maxTimeForSpinUp = 2;
    private int launchRemaining = 0;

    private String[] side;
    public static final double ON               = 1.0;
    public static final double OFF              = 0.0;
    double LAUNCHER_CLOSE_TARGET_VELOCITY = 1140; //in ticks/second for the close goal. //1200 original
    final double LAUNCHER_CLOSE_MIN_VELOCITY = 1120; //minimum required to start a shot for close goal.

    final double LAUNCHER_FAR_TARGET_VELOCITY = 1350; //Target velocity for far goal
    final double LAUNCHER_FAR_MIN_VELOCITY = 1325; //minimum required to start a shot for far goal.

    private ElapsedTime stateTimer = new ElapsedTime();

    private enum LauncherState{
        IDLE,
        SPIN_UP,
        LEFTLAUNCH,
        RIGHTLAUNCH,
        BOTHLAUNCH,
        RESET
    }

    private LauncherState launcherState;
    public void init(HardwareMap hardwareMap){
        rightLauncher = hardwareMap.get(DcMotorEx.class, "right_launcher");
        leftLauncher = hardwareMap.get(DcMotorEx.class, "left_launcher");
        transfer = hardwareMap.get(DcMotorEx.class, "transfer");
        rightFeeder = hardwareMap.get(CRServo.class, "right_feeder");
        leftFeeder = hardwareMap.get(CRServo.class, "left_feeder");

        leftLauncher.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftLauncher.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leftLauncher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));
        leftLauncher.setDirection(DcMotorSimple.Direction.FORWARD);
        leftLauncher.setPower(0);

        rightLauncher.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightLauncher.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightLauncher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));
        rightLauncher.setDirection(DcMotorSimple.Direction.REVERSE);
        rightLauncher.setPower(0);

        launcherState = LauncherState.IDLE;
    }

    public void update(){
        switch(launcherState){
            case IDLE:
                if(launchRemaining > 0){
                    //move the set velocity outside of if statement if needed
                    leftLauncher.setVelocity(LAUNCHER_CLOSE_TARGET_VELOCITY);
                    rightLauncher.setVelocity(LAUNCHER_CLOSE_TARGET_VELOCITY);
                    stateTimer.reset();
                    launcherState = LauncherState.SPIN_UP;
                }
                break;
            case SPIN_UP:
                if(side[launchRemaining-1].equals("left")){
                    if(leftLauncher.getVelocity() > LAUNCHER_CLOSE_MIN_VELOCITY || stateTimer.seconds() > maxTimeForSpinUp){
                        leftFeeder.setPower(ON);
                        stateTimer.reset();
                        launcherState = LauncherState.LEFTLAUNCH;
                    }
                }else if(side[launchRemaining-1].equals("right")){
                    if(rightLauncher.getVelocity() > LAUNCHER_CLOSE_MIN_VELOCITY || stateTimer.seconds() > maxTimeForSpinUp){
                        rightFeeder.setPower(ON);
                        stateTimer.reset();
                        launcherState = LauncherState.RIGHTLAUNCH;
                    }
                }else{
                    //both
                    if(rightLauncher.getVelocity() > LAUNCHER_CLOSE_MIN_VELOCITY || stateTimer.seconds() > maxTimeForSpinUp){
                        leftFeeder.setPower(ON);
                        rightFeeder.setPower(ON);
                        stateTimer.reset();
                        launcherState = LauncherState.BOTHLAUNCH;
                    }
                }
                break;
            case LEFTLAUNCH:
                if(stateTimer.seconds()>launchTime){
                    launchRemaining--;
                    leftFeeder.setPower(OFF);
                    stateTimer.reset();
                    launcherState = LauncherState.RESET;
                }
                break;
            case RIGHTLAUNCH:
                if(stateTimer.seconds()>launchTime){
                    launchRemaining--;
                    rightFeeder.setPower(OFF);
                    stateTimer.reset();
                    launcherState = LauncherState.RESET;
                }
                break;
            case BOTHLAUNCH:
                if(stateTimer.seconds()>launchTime){
                    launchRemaining--;
                    rightFeeder.setPower(OFF);
                    leftFeeder.setPower(OFF);
                    stateTimer.reset();
                    launcherState = LauncherState.RESET;
                }
                break;
            case RESET:
                if(stateTimer.seconds()>gapBetweenLaunch){
                    if(launchRemaining > 0){
                        stateTimer.reset();
                        launcherState = LauncherState.SPIN_UP;
                    }else{
                        leftLauncher.setVelocity(0);
                        rightLauncher.setVelocity(0);
                        launcherState = LauncherState.IDLE;
                    }
                }
                break;

        }
    }

    public void shoot(int numberOfShots, String[] pattern){
        if(launcherState == LauncherState.IDLE){
            launchRemaining = numberOfShots;
            side = pattern;
        }
    }
    public boolean isBusy(){
        return launcherState != LauncherState.IDLE;
    }

}
