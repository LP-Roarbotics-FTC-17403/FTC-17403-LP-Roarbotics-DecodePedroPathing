/*
    CREDITS: Pedro Pathing visit https://pedropathing.com/docs/pathing/examples/auto
 */

package org.firstinspires.ftc.teamcode.Auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.SubSystems.CameraSystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(group = "Blue")
public class BlueSidePedroPathing extends OpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private LauncherStateMachine launcher = new LauncherStateMachine();
    private int tagNumber = 21;
    private boolean aprilTagDone = false;
    private String[] pattern;
    private String[] GPP = {"right", "right", "left"};
    private String[] PGP = {"right", "left", "right"};
    private String[] PPG = {"left", "right", "right"};
    private boolean shotTriggered = false;

    CameraSystem camera;
    private int pathState;
    //Set the different points robots may travel to
    private final Pose startPose = new Pose(32.90173410404624, 0, Math.toRadians(0));
    private final Pose scorePose = new Pose(0, 0, Math.toRadians(0));
    private final Pose pickup1Pose = new Pose(0, 0, Math.toRadians(0));
    private final Pose gateOpenPose = new Pose();
    private final Pose pickup2Pose = new Pose(0, 0, Math.toRadians(0));
    private final Pose pickup3Pose = new Pose(0, 0, Math.toRadians(0));

    //Different paths, Path for singular pathing, path chain for paths following one after another
    private PathChain readAprilTag,scorePreload, grabPickup1, scorePickup1, openGate, grabPickup2, scorePickup2, grabPickup3, scorePickup3, endPath;

    //this is where you build all your paths
    public void buildPaths(){
        readAprilTag = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(32.902, 135.769),

                                new Pose(49.341, 113.202)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(60))

                .build();

        scorePreload = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(49.341, 113.202),

                                new Pose(59.884, 83.457)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(60), Math.toRadians(130))

                .build();

        grabPickup1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(59.884, 83.457),

                                new Pose(31.301, 80.763)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(130), Math.toRadians(180)).addPath(
                        new BezierCurve(
                                new Pose(31.301, 80.763),
                                new Pose(39.176, 88.526),
                                new Pose(20.035, 86.393)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        openGate = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(20.035, 86.393),
                                new Pose(35.295, 71.347),
                                new Pose(17.387, 72.821)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                .build();

        scorePickup1 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(17.387, 72.821),
                                new Pose(38.390, 79.286),
                                new Pose(59.890, 83.347)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(130))

                .build();

        grabPickup2 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(59.890, 83.347),
                                new Pose(57.986, 55.913),
                                new Pose(30.289, 57.133)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(130), Math.toRadians(180)).addPath(
                        new BezierCurve(
                                new Pose(30.289, 57.133),
                                new Pose(37.260, 65.315),
                                new Pose(22.751, 63.832)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        scorePickup2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(22.751, 63.832),

                                new Pose(59.988, 83.277)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(130))

                .build();

        grabPickup3 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(59.988, 83.277),
                                new Pose(65.256, 32.955),
                                new Pose(29.994, 32.428)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(130), Math.toRadians(180)).addPath(
                        new BezierCurve(
                                new Pose(29.994, 32.428),
                                new Pose(43.543, 42.272),
                                new Pose(12.399, 37.457)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        scorePickup3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(12.399, 37.457),

                                new Pose(59.723, 83.382)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(130))

                .build();

        endPath = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(59.723, 83.382),

                                new Pose(45.873, 69.486)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(130))

                .build();
    }

    //this is the singular method that calls every path in auto
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(readAprilTag);
                actionTimer.resetTimer();
                setPathState(1);
                break;
            case 1:
            /* You could check for
            - Follower State: "if(!follower.isBusy()) {}"
            - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
            - Robot Position: "if(follower.getPose().getX() > 36) {}"
            */
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    if(actionTimer.getElapsedTimeSeconds() < 2){
                        tagNumber = camera.getPattern();
                        if(tagNumber == 21){
                            pattern = GPP;
                        }else if(tagNumber == 22){
                            pattern = PGP;
                        }else if(tagNumber == 23){
                            pattern = PPG;
                        }
                    }else{
                        aprilTagDone = true;
                    }
                    /* Score Preload */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    if(aprilTagDone){
                        follower.followPath(scorePreload,true);
                        setPathState(2);
                    }
                }
                break;
            case 2:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if(!follower.isBusy()) {
                    if(!shotTriggered){
                        launcher.shoot(3, pattern);
                        shotTriggered = true;
                    }else if(shotTriggered && !launcher.isBusy()){
                        follower.followPath(grabPickup1, true);
                        setPathState(3);
                        shotTriggered = false;
                        actionTimer.resetTimer();
                    }
                }
                break;
            case 3:
                if(!follower.isBusy()) {
                        follower.followPath(openGate, true);
                        setPathState(4);
                }
                break;
            case 4:
                if(!follower.isBusy()){
                    if (actionTimer.getElapsedTimeSeconds() > 1) {
                        follower.followPath(scorePickup1, true);
                        setPathState(5);
                    }
                }
                break;
            case 5:
                if(!follower.isBusy()) {
                    if(!shotTriggered){
                        launcher.shoot(3, pattern);
                        shotTriggered = true;
                    }else if(shotTriggered && !launcher.isBusy()){
                        follower.followPath(grabPickup2, true);
                        setPathState(6);
                        shotTriggered = false;
                        actionTimer.resetTimer();
                    }
                }
                break;
            case 6:
                if(!follower.isBusy()) {
                    follower.followPath(scorePickup2, true);
                    setPathState(7);
                }
                break;
            case 7:
                if(!follower.isBusy()) {
                    if(!shotTriggered){
                        launcher.shoot(3, pattern);
                        shotTriggered = true;
                    }else if(shotTriggered && !launcher.isBusy()){
                        follower.followPath(grabPickup3, true);
                        setPathState(8);
                        shotTriggered = false;
                        actionTimer.resetTimer();
                    }
                }
                break;
            case 8:
                if(!follower.isBusy()) {
                    follower.followPath(scorePickup3, true);
                    setPathState(9);
                }
                break;
            case 9:
                if(!follower.isBusy()) {
                    if(!shotTriggered){
                        launcher.shoot(3, pattern);
                        shotTriggered = true;
                    }else if(shotTriggered && !launcher.isBusy()){
                        follower.followPath(endPath, true);
                        setPathState(10);
                        shotTriggered = false;
                        actionTimer.resetTimer();
                    }
                }
                break;
            case 10:
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
        //this continuously updates which path to run
        autonomousPathUpdate();
    }
    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        camera = new CameraSystem(hardwareMap);
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer.resetTimer();
        camera.cameraOn();
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
        launcher.init(hardwareMap);
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