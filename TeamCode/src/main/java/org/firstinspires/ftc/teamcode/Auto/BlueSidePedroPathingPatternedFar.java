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

import org.firstinspires.ftc.teamcode.SubSystems.CameraSystem;
import org.firstinspires.ftc.teamcode.SubSystems.Hammer;
import org.firstinspires.ftc.teamcode.SubSystems.Inhaler;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(group = "Blue")
public class BlueSidePedroPathingPatternedFar extends OpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private LauncherStateMachine launcher = new LauncherStateMachine();
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

    private CameraSystem camera;
    Inhaler inhaler;
    Inhaler transfer;

    private Hammer hammer;

    private int pathState;
    //Set the different points robots may travel to
    private final Pose startPose = new Pose(57.212, 9.038, Math.toRadians(90));
    private final Pose scorePose = new Pose(0, 0, Math.toRadians(0));
    private final Pose pickup1Pose = new Pose(0, 0, Math.toRadians(0));
    private final Pose gateOpenPose = new Pose();
    private final Pose pickup2Pose = new Pose(0, 0, Math.toRadians(0));
    private final Pose pickup3Pose = new Pose(0, 0, Math.toRadians(0));

    //Different paths, Path for singular pathing, path chain for paths following one after another
    private PathChain scorePreload, endPath;

    //this is where you build all your paths
    public void buildPaths(){
        scorePreload = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(57.212, 9.038),

                                new Pose(59.481, 21.635)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(110))

                .build();

        endPath = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(59.481, 21.635),

                                new Pose(15.404, 13.327)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(90))

                .build();
    }

    //this is the singular method that calls every path in auto
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                inhaler.inhale_on();
                actionTimer.resetTimer();
                int tempPattern = camera.getPattern();
                if(tempPattern != 0){
                    tagNumber = tempPattern;
                    if(tagNumber == 21){
                        pattern = GPP;
                    }else if(tagNumber == 22){
                        pattern = PGP;
                    }else if(tagNumber == 23){
                        pattern = PPG;
                    }
                    aprilTagDone = true;
                }else if(actionTimer.getElapsedTimeSeconds()>5){
                    aprilTagDone = true;
                }
                /* Score Preload */
                /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                if(aprilTagDone){
                    follower.followPath(scorePreload,true);
                    setPathState(1);
                }
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
                        launcher.shoot(3, pattern, tagNumber);
                        shotTriggered = true;
                    }else if(shotTriggered && !launcher.isBusy()){
                        follower.followPath(endPath, true);
                        transfer.inhale_off();
                        setPathState(2);
                        shotTriggered = false;
                        actionTimer.resetTimer();
                    }
                }
                break;
            case 2:
                inhaler.inhale_off();
                launcher.launcherOff();
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
        telemetry.addData("current tag id: ", tagNumber);
        telemetry.update();
        //this continuously updates which path to run
        autonomousPathUpdate();
    }
    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        camera = new CameraSystem(hardwareMap);
        transfer = new Inhaler(hardwareMap, "transfer");
        inhaler = new Inhaler(hardwareMap, "intake");
        hammer = new Hammer(hardwareMap, "diverter");
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer.resetTimer();
        camera.cameraOn();
        follower = Constants.createFollower(hardwareMap);
        inhaler.initialize(true);
        transfer.initialize(true);
        hammer.initialize();
        buildPaths();
        follower.setStartingPose(startPose);
        launcher.init(hardwareMap);
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