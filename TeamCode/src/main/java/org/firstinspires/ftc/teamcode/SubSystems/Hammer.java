package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Hammer {
    private Servo hammer = null;
    final double LEFT_POSITION = 0.33; //the left and right position for the diverter servo
    //changed from 0.33
    final double RIGHT_POSITION = 0.035;

    final double middlePosition = 0.18;  //changed from 0.2

    private double closePosition = 0;

    private double openPosition = 0;

    public Hammer(HardwareMap hardwareMap, String deviceName){
        this.hammer = hardwareMap.get(Servo.class, deviceName);
    }

    public void initialize(){
        hammer.setPosition(LEFT_POSITION);
    }


    public void setRight(){
        hammer.setPosition(RIGHT_POSITION);
    }

    public void setLeft(){
        hammer.setPosition(LEFT_POSITION);
    }

    public void setMiddle(){
        hammer.setPosition(middlePosition);
    }

    public void setPosition(double closePosition, double openPosition){
        this.closePosition = closePosition;
        this.openPosition = openPosition;
    }


}
