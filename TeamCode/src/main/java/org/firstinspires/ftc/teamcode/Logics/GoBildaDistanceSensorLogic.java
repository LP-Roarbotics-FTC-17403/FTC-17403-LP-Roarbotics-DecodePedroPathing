package org.firstinspires.ftc.teamcode.Logics;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.SubSystems.Hammer;
import org.firstinspires.ftc.teamcode.SubSystems.LED;

public class GoBildaDistanceSensorLogic {
    private AnalogInput laserDistance;
    private AnalogInput laserDistance2;
    private double distance = 0;
    private double distance2 = 0;

    private static final double MAX_VOLTS = 3.3;
    private static final double MAX_DISTANCE_MM = 1000.0;

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
        laserDistance = hardwareMap.get(AnalogInput.class,"distance_sensor");
        laserDistance2 = hardwareMap.get(AnalogInput.class,"distance_sensor2");
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

                distance = ((laserDistance.getVoltage()) / MAX_VOLTS) * MAX_DISTANCE_MM;
                distance2 = ((laserDistance2.getVoltage()) / MAX_VOLTS) * MAX_DISTANCE_MM;

                if(distance < 275 || distance2 < 275){
                    stateTimer.reset();
                    detectionState = DetectionState.confirming;
                }
                currentStep = "detecting";
                break;
            case confirming:
                distance = ((laserDistance.getVoltage()) / MAX_VOLTS) * MAX_DISTANCE_MM;
                distance2 = ((laserDistance2.getVoltage()) / MAX_VOLTS) * MAX_DISTANCE_MM;
                if(distance >= 275 && distance2 >= 275){
                    detectionState = DetectionState.detecting;
                    if(divertOn) {
                        diverter.slap();
                    }
                }
                if((distance < 275 || distance2 < 275) && stateTimer.seconds() > 1){
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

    public double getDistance(){
        return distance;
    }
    public void divertRight(){
        diverter.setRight();
    }
    public void divertLeft(){
        diverter.setLeft();
    }
}

