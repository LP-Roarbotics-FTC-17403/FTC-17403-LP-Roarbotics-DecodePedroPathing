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

    private int tagNumber = 21;
    private double launchTime = 0.6;

    private double longTime = 0.7;
    private double shortime = 0.3;

    private double gapBetweenLaunch = 0.3;
    private double maxTimeForSpinUp = 0.4;
    private int launchRemaining = 0;



    private String[] side;
    public static final double ON               = 1.0;
    public static final double OFF              = 0.0;
    double LAUNCHER_CLOSE_TARGET_VELOCITY = 1140; //in ticks/second for the close goal. //1200 original
    final double LAUNCHER_CLOSE_MIN_VELOCITY = 1120; //minimum required to start a shot for close goal.

    final double LAUNCHER_FAR_TARGET_VELOCITY = 1350; //Target velocity for far goal
    final double LAUNCHER_FAR_MIN_VELOCITY = 1325; //minimum required to start a shot for far goal.

    double desiredTargetVelocity = 1140;
    double desiredMinVelocity = 1120;

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
        leftFeeder.setDirection(DcMotorSimple.Direction.REVERSE);
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
                leftLauncher.setVelocity(desiredTargetVelocity);
                rightLauncher.setVelocity(desiredTargetVelocity);
                if(launchRemaining > 0){
                    //move the set velocity outside of if statement if needed
                    leftLauncher.setVelocity(desiredTargetVelocity);
                    rightLauncher.setVelocity(desiredTargetVelocity);
                    stateTimer.reset();
                    launcherState = LauncherState.SPIN_UP;
                }
                break;
            case SPIN_UP:
                if(side[launchRemaining-1].equals("right")){ //changed direction
                    if(leftLauncher.getVelocity() > desiredMinVelocity || stateTimer.seconds() > maxTimeForSpinUp){
                        leftFeeder.setPower(ON);
                        stateTimer.reset();
                        launcherState = LauncherState.LEFTLAUNCH; // changed opposite
                    }
                }else if(side[launchRemaining-1].equals("left")){
                    if(rightLauncher.getVelocity() > desiredMinVelocity || stateTimer.seconds() > maxTimeForSpinUp){
                        rightFeeder.setPower(ON);
                        stateTimer.reset();
                        launcherState = LauncherState.RIGHTLAUNCH; // changed opposite
                    }
                }else{
                    //both
                    if(rightLauncher.getVelocity() > desiredMinVelocity || stateTimer.seconds() > maxTimeForSpinUp){
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
                        if(launchRemaining < 3){
                            launchTime = longTime;
                        }
                    }else{
                        leftLauncher.setVelocity(desiredTargetVelocity);
                        rightLauncher.setVelocity(desiredTargetVelocity);
                        launcherState = LauncherState.IDLE;
                    }
                }
                break;

        }
    }

    public void shoot(int numberOfShots, String[] pattern, int tagNumber){
        if(launcherState == LauncherState.IDLE){
            launchRemaining = numberOfShots;
            side = pattern;
            if(tagNumber == 22){
                launchTime = shortime;
            }
        }
    }
    public boolean isBusy(){
        return launcherState != LauncherState.IDLE;
    }

    public void setClose(){
        desiredTargetVelocity = LAUNCHER_CLOSE_TARGET_VELOCITY;
        desiredMinVelocity = LAUNCHER_CLOSE_MIN_VELOCITY;
    }

    public void setFar(){
        desiredTargetVelocity = LAUNCHER_FAR_TARGET_VELOCITY;
        desiredMinVelocity = LAUNCHER_FAR_MIN_VELOCITY;
    }

    public int currentRemainingShot(){
        return launchRemaining;
    }

    public void setLaunchTime(int time){
        launchTime = time;
    }

    public void launcherOff(){
        rightLauncher.setVelocity(0);
        leftLauncher.setVelocity(0);
    }

}
