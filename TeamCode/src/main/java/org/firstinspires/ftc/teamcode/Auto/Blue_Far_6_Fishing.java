/*
    CREDITS: Pedro Pathing visit https://pedropathing.com/docs/pathing/examples/auto
 */

package org.firstinspires.ftc.teamcode.Auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Logics.GoBildaDistanceSensorLogic;
import org.firstinspires.ftc.teamcode.Logics.LauncherStateMachine;
import org.firstinspires.ftc.teamcode.Logics.RapidFireStateMachine;
import org.firstinspires.ftc.teamcode.SubSystems.CameraSystem;
import org.firstinspires.ftc.teamcode.SubSystems.Hammer;
import org.firstinspires.ftc.teamcode.SubSystems.Inhaler;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(group = "Blue")
public class Blue_Far_6_Fishing extends OpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private RapidFireStateMachine launcher = new RapidFireStateMachine();
    private Servo rightGate;
    private Servo leftGate;
    private int tagNumber = 21;
    private boolean aprilTagDone = false;
    private String[] pattern;
    private String[] GPP = {"right", "right", "left"};
    private String[] PGP = {"right", "left", "right"};
    private String[] PPG = {"left", "right", "right"};

    private String[] Preload;
    private String[] first;
    private String[] second;
    private boolean shotTriggered = false;
    final double LEFT_OPEN_POSITION = 0;
    final double LEFT_CLOSE_POSITION = 0.175;

    final double RIGHT_CLOSE_POSITION = 0.05;
    final double RIGHT_OPEN_POSITION = 0.225;
    private GoBildaDistanceSensorLogic distance = new GoBildaDistanceSensorLogic();

    private CameraSystem camera;
    Inhaler inhaler;
    Inhaler transfer;


    private int pathState;
    //Set the different points robots may travel to
    private final Pose startPose = new Pose(57.212, 9.038, Math.toRadians(90));
    private final Pose scorePose = new Pose(0, 0, Math.toRadians(0));
    private final Pose pickup1Pose = new Pose(0, 0, Math.toRadians(0));
    private final Pose gateOpenPose = new Pose();
    private final Pose pickup2Pose = new Pose(0, 0, Math.toRadians(0));
    private final Pose pickup3Pose = new Pose(0, 0, Math.toRadians(0));

    //Different paths, Path for singular pathing, path chain for paths following one after another
    private PathChain scorePreload, grabPickup1p1, grabPickup1p2, scorePickup1, grabPickup2p1, oneToTwo, twoToOne, oneScorePickup2, twoScorePickup2, endPath;

    //this is where you build all your paths
    public void buildPaths(){
        scorePreload = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(57.212, 9.038),

                                new Pose(59.481, 21.635)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(110))

                .build();

        grabPickup1p1 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(59.481, 21.635),
                                new Pose(60.437, 33.693),
                                new Pose(41.740, 32.110)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        grabPickup1p2 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(41.740, 32.110),
                                new Pose(57.702, 38.818),
                                new Pose(24.491, 35.873)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        scorePickup1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(24.491, 35.873),

                                new Pose(59.481, 21.635)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(110))

                .build();

        grabPickup2p1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(59.481, 21.635),

                                new Pose(12.116, 23.711)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        oneToTwo = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(12.116, 23.711),
                                new Pose(59.225, 18.465),
                                new Pose(14.069, 11.393)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();
        twoToOne = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(14.069, 11.393),
                                new Pose(59.225, 18.465),
                                new Pose(12.116, 23.711)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();
        twoScorePickup2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(14.069, 11.393),

                                new Pose(59.481, 21.635)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(110))

                .build();
        oneScorePickup2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(12.116, 23.711),

                                new Pose(59.481, 21.635)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(110))

                .build();

        endPath = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(59.481, 21.635),

                                new Pose(48.260, 20.428)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(110))

                .build();
    }

    //this is the singular method that calls every path in auto
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                opmodeTimer.resetTimer();
                follower.followPath(scorePreload,true);
                rightGate.setPosition(RIGHT_OPEN_POSITION);
                leftGate.setPosition(LEFT_OPEN_POSITION);
                setPathState(1);
                break;
            case 1:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if(!follower.isBusy()) {
                    if(tagNumber != 22){
                        transfer.inhale_on();
                    }else if(launcher.currentRemainingShot() < 2){
                        transfer.inhale_on();
                    }
                    if(!shotTriggered){
                        launcher.shoot(3);
                        shotTriggered = true;
                    }else if(shotTriggered && !launcher.isBusy()){
                        follower.followPath(grabPickup1p1, true);
                        transfer.inhale_off();
                        setPathState(2);
                        shotTriggered = false;
                        distance.divertRight();
                    }
                }
                break;
            case 2:
                if(!follower.isBusy()){
                    if (actionTimer.getElapsedTimeSeconds() > 1) {
                        follower.followPath(grabPickup1p2, true);
                        setPathState(3);
                        distance.divertLeft();
                    }
                }
                break;
            case 3:
                if(!follower.isBusy()){
                    if (actionTimer.getElapsedTimeSeconds() > 1) {
                        follower.followPath(scorePickup1, true);
                        setPathState(4);
                    }
                }
                break;
            case 4:
                if(!follower.isBusy()) {
                    if(tagNumber != 22){
                        transfer.inhale_on();
                    }else if(launcher.currentRemainingShot() < 2){
                        transfer.inhale_on();
                    }
                    if(!shotTriggered){
                        launcher.shoot(3);
                        shotTriggered = true;
                    }else if(shotTriggered && !launcher.isBusy()){
                        follower.followPath(grabPickup2p1, true);
                        distance.startDetect();
                        transfer.inhale_on();
                        rightGate.setPosition(RIGHT_CLOSE_POSITION);
                        leftGate.setPosition(LEFT_CLOSE_POSITION);
                        setPathState(5);
                        shotTriggered = false;
                        actionTimer.resetTimer();
                        distance.divertRight();
                    }
                }
                break;
            case 5:
                if(!follower.isBusy()) {
                    if(distance.loaded() || opmodeTimer.getElapsedTimeSeconds() > 23){
                        follower.followPath(oneScorePickup2, true);
                        setPathState(7);
                    }else{
                        follower.followPath(oneToTwo, true);
                        setPathState(6);
                    }
                }
                break;
            case 6:
                if(!follower.isBusy()) {
                    if(distance.loaded() || opmodeTimer.getElapsedTimeSeconds() > 23){
                        follower.followPath(twoScorePickup2, true);
                        setPathState(7);
                    }else{
                        follower.followPath(twoToOne, true);
                        setPathState(5);
                    }

                }
                break;
            case 7:
                if(!follower.isBusy()) {
                    if(!shotTriggered){
                        leftGate.setPosition(LEFT_OPEN_POSITION);
                        rightGate.setPosition(RIGHT_OPEN_POSITION);
                        launcher.shoot(3);
                        shotTriggered = true;
                    }else if(shotTriggered && !launcher.isBusy()){
                        follower.followPath(endPath, true);
                        transfer.inhale_off();
                        setPathState(8);
                        shotTriggered = false;
                        actionTimer.resetTimer();
                    }
                }
                break;
            case 8:
                break;

        }
    }
    //this is what you use to update the desired state
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
        shotTriggered = false;
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {
        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        launcher.update();
        distance.update(true);

        //this continuously updates which path to run
        autonomousPathUpdate();
    }
    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        camera = new CameraSystem(hardwareMap);
        transfer = new Inhaler(hardwareMap, "transfer");
        inhaler = new Inhaler(hardwareMap, "intake");
        rightGate = hardwareMap.get(Servo.class, "right_gate");
        leftGate = hardwareMap.get(Servo.class, "left_gate");
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer.resetTimer();
        camera.cameraOn();
        follower = Constants.createFollower(hardwareMap);
        inhaler.initialize(true);
        transfer.initialize(true);
        buildPaths();
        follower.setStartingPose(startPose);
        launcher.init(hardwareMap);
        distance.init(hardwareMap);
        launcher.setFar();
    }
    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}
    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        actionTimer.resetTimer();
        setPathState(0);
    }
}