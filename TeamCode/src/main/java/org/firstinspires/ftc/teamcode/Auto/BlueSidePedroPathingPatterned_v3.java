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
public class BlueSidePedroPathingPatterned_v3 extends OpMode {
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
    private final Pose startPose = new Pose(33.942, 133.896, Math.toRadians(90));
    private final Pose scorePose = new Pose(0, 0, Math.toRadians(0));
    private final Pose pickup1Pose = new Pose(0, 0, Math.toRadians(0));
    private final Pose gateOpenPose = new Pose();
    private final Pose pickup2Pose = new Pose(0, 0, Math.toRadians(0));
    private final Pose pickup3Pose = new Pose(0, 0, Math.toRadians(0));

    //Different paths, Path for singular pathing, path chain for paths following one after another
    private PathChain readAprilTag,scorePreload, grabPickup1p1, grabPickup1p2, scorePickup1, grabPickup2p1, grabPickup2p2, grabPickup2p3, scorePickup2, grabPickup3p1, grabPickup3p2, scorePickup3, endPath;

    //this is where you build all your paths
    public void buildPaths(){
        readAprilTag = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(33.942, 133.896),

                                new Pose(49.341, 113.202)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(60))

                .build();

        scorePreload = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(49.341, 113.202),

                                new Pose(59.884, 83.457)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(60), Math.toRadians(133))

                .build();

        grabPickup1p1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(59.884, 83.457),

                                new Pose(38.549, 85.636)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))
                //.setTangentHeadingInterpolation()

                .build();

        grabPickup1p2 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(38.549, 85.636),
                                new Pose(65.986, 76.566),
                                new Pose(22.324, 84.601)   //from 83.601
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        scorePickup1 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(22.324, 83.601),
                                new Pose(38.390, 79.286),
                                new Pose(59.890, 83.347)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))

                .build();

        grabPickup2p1 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(59.890, 83.347),
                                new Pose(55.696, 49.462),
                                new Pose(41.740, 57.133)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))

                .build();

        grabPickup2p2 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(41.740, 57.133),
                                new Pose(55.780, 64.899),
                                new Pose(36.266, 62.561)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        grabPickup2p3 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(36.266, 62.561),
                                new Pose(50.716, 54.505), //35 to 50
                                new Pose(27.440, 61.312) //60 to 61
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        scorePickup2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(27.440, 58.312),

                                new Pose(59.988, 83.277)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))

                .build();

        grabPickup3p1 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(59.988, 83.277),
                                new Pose(57.283, 24.243),
                                new Pose(41.740, 32.110)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))

                .build();

        grabPickup3p2 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(41.740, 32.110),
                                new Pose(57.702, 38.818),
                                new Pose(24.491, 35.873)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        endPath = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(59.988, 83.277),

                                new Pose(45.873, 69.486)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();
    }

    //this is the singular method that calls every path in auto
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                inhaler.inhale_on();
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
                    if(tagNumber != 22){
                        transfer.inhale_on();
                    }else if(launcher.currentRemainingShot() < 2){
                        transfer.inhale_on();
                    }
                    if(!shotTriggered){
                        launcher.shoot(3, pattern, tagNumber);
                        shotTriggered = true;
                    }else if(shotTriggered && !launcher.isBusy()){
                        follower.followPath(grabPickup1p1, true);
                        transfer.inhale_off();
                        setPathState(3);
                        shotTriggered = false;
                        actionTimer.resetTimer();
                    }
                }
                break;
            case 3:
                if(!follower.isBusy()){
                    if (actionTimer.getElapsedTimeSeconds() > 1) {
                        hammer.setMiddle();
                        follower.followPath(grabPickup1p2, true);
                        setPathState(4);
                    }
                }
            case 4:
                if(!follower.isBusy()){
                    if (actionTimer.getElapsedTimeSeconds() > 1) {
                        hammer.setLeft();
                        follower.followPath(scorePickup1, true);
                        setPathState(5);
                    }
                }
                break;
            case 5:
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
                        follower.followPath(grabPickup2p1, true);
                        transfer.inhale_off();
                        setPathState(6);
                        shotTriggered = false;
                        actionTimer.resetTimer();
                    }
                }
                break;
            case 6:
                if(!follower.isBusy()) {
                    hammer.setRight();
                    transfer.inhale_on();
                    follower.followPath(grabPickup2p2, true);
                    setPathState(7);
                }
                break;
            case 7:
                if(!follower.isBusy()) {
                    hammer.setLeft();
                    follower.followPath(grabPickup2p3, true);
                    transfer.inhale_off();
                    setPathState(8);
                }
                break;
            case 8:
                if(!follower.isBusy()) {
                    follower.followPath(scorePickup2, true);
                    setPathState(9);
                }
                break;
            case 9:
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
                        follower.followPath(grabPickup3p1, true);
                        transfer.inhale_off();
                        setPathState(10);
                        shotTriggered = false;
                        hammer.setRight();
                        actionTimer.resetTimer();
                    }
                }
                break;
            case 10:
                if(!follower.isBusy()){
                    if (actionTimer.getElapsedTimeSeconds() > 1) {
                        hammer.setLeft();
                        follower.followPath(grabPickup3p2, true);
                        setPathState(11);
                    }
                }
            case 11:
                if(!follower.isBusy()){
                    if (actionTimer.getElapsedTimeSeconds() > 1) {
                        follower.followPath(endPath, true);
                        setPathState(12);
                    }
                }
                break;
            case 12:
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
        launcher.setClose();
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