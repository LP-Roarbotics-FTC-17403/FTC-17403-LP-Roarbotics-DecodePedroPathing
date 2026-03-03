package org.firstinspires.ftc.teamcode.Logics;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.SubSystems.Hammer;
import org.firstinspires.ftc.teamcode.SubSystems.LED;

public class RevV3ColorSensorLogic {
    RevColorSensorV3 leftColorSensor;
    RevColorSensorV3 rightColorSensor;
    private double leftDistance = 0;
    private double rightDistance = 0;


    private Hammer diverter;
    private LED leftLight;
    private LED rightLight;

    private String currentStep = "Not Detecting";

    boolean detecting = false;


    private ElapsedTime stateTimer = new ElapsedTime();

    private enum DetectionState{
        notDetecting,
        detecting,
        confirming,
        readyToGo
    }
    DetectionState detectionState;
    public void init(HardwareMap hardwareMap) {
        leftColorSensor = hardwareMap.get(RevColorSensorV3.class, "left_color_sensor");
        rightColorSensor = hardwareMap.get(RevColorSensorV3.class, "right_color_sensor");
        leftLight = new LED(hardwareMap, "left_light");
        rightLight = new LED(hardwareMap, "right_light");
        diverter = new Hammer(hardwareMap, "diverter");
        detectionState = DetectionState.notDetecting;
    }

    public void update(boolean divertOn) {
        switch(detectionState){
            case notDetecting:
                if(detecting){
                    detectionState = DetectionState.detecting;
                    diverter.setLeft();
                }
                currentStep = "Not detecting";
                break;
            case detecting:
                leftLight.lightOff();
                rightLight.lightOff();

                leftDistance = leftColorSensor.getDistance(DistanceUnit.CM);
                rightDistance = rightColorSensor.getDistance(DistanceUnit.CM);

                if(leftDistance < 8 || rightDistance < 8){
                    stateTimer.reset();
                    detectionState = DetectionState.confirming;
                }
                currentStep = "detecting";
                break;
            case confirming:
                leftDistance = leftColorSensor.getDistance(DistanceUnit.CM);
                rightDistance = rightColorSensor.getDistance(DistanceUnit.CM);
                if(leftDistance >= 8 && rightDistance >= 8){
                    detectionState = DetectionState.detecting;
                    if(divertOn) {
                        diverter.slap();
                    }
                }
                if((leftDistance < 9 || rightDistance < 9) && stateTimer.seconds() > 1){
                    detectionState = DetectionState.readyToGo;
                }
                currentStep = "confirming for full intake";
                break;
            case readyToGo:
                leftLight.setGreen();
                rightLight.setGreen();
                detectionState = DetectionState.notDetecting;
                detecting = false;
                break;
        }
    }
    public boolean loaded(){
        if(detectionState == DetectionState.notDetecting){
            return true;
        }else{
            return false;
        }
    }

    public void startDetect(){
        detecting = true;
    }
    public String currentMessage(){
        return currentStep;
    }

    public double getLeftDistance() {
        return leftDistance;
    }

    public double getRightDistance() {
        return rightDistance;
    }
}

