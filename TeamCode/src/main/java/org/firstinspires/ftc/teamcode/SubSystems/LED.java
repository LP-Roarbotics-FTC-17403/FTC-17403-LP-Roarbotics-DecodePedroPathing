package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class LED {
    private Servo light;
    public LED(HardwareMap hardwareMap,String deviceName){
        light = hardwareMap.get(Servo.class, deviceName);
    }

    public void setGreen(){
        light.setPosition(0.500);
    }
    public void setPurple(){
        light.setPosition(0.722);
    }

    public void setRed(){
        light.setPosition(0.28);
    }
    public void lightOff(){
        light.setPosition(0);
    }
}
