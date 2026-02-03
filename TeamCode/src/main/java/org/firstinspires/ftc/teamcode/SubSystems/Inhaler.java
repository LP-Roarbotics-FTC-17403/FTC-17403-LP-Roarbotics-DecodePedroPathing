package org.firstinspires.ftc.teamcode.SubSystems;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Inhaler {
    private DcMotorEx inhale = null;

    public static final double INHALE_ON                = 1.0;
    public static final double INHALE_OFF               = 0.0;


    public Inhaler(HardwareMap hardwareMap, String deviceName){
        this.inhale = hardwareMap.get(DcMotorEx.class, deviceName);
    }
    public void initialize(boolean reverse){
        if(reverse){
            inhale.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        inhale.setPower(INHALE_OFF);
    }

    public void inhale_on(){
        inhale.setPower(INHALE_ON);
    }
    public void inhale_off(){
        inhale.setPower(INHALE_OFF);
    }


}
