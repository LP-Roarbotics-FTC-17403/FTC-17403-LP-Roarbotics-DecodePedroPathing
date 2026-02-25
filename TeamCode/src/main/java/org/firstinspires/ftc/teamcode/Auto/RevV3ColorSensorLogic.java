package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.SubSystems.LED;

public class RevV3ColorSensorLogic {
    RevColorSensorV3 leftColorSensor;
    RevColorSensorV3 rightColorSensor;
    private double leftDistance = 0;
    private double rightDistance = 0;

    private LED leftLight;
    private LED rightLight;

    private

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
        rightColorSensor = hardwareMap.get(RevColorSensorV3.class, "left_color_sensor");
        leftLight = new LED(hardwareMap, "left_light");
        rightLight = new LED(hardwareMap, "right_light");
        detectionState = DetectionState.notDetecting;
    }

    public void update() {
        switch(detectionState){
            case notDetecting:
                if(detecting){
                    detectionState = DetectionState.detecting;
                }
                break;
            case detecting:
                leftDistance = leftColorSensor.getDistance(DistanceUnit.CM);
                rightDistance = rightColorSensor.getDistance(DistanceUnit.CM);
                stateTimer.reset();
                detectionState = DetectionState.confirming;
                break;
            case confirming:
                if(leftDistance >= 8 || rightDistance >= 8){
                    detectionState = DetectionState.detecting;
                }
                if((leftDistance < 8 || rightDistance < 8) && stateTimer.seconds() > 2){
                    detectionState = DetectionState.readyToGo;
                }
                break;
            case readyToGo:
                leftLight.setGreen();
                rightLight.setGreen();
                detectionState = DetectionState.notDetecting;
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
}

