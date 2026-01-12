package org.firstinspires.ftc.teamcode.Modules.Shooter;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Robot.Hardware;

public class Shooter {


    DcMotorEx motor;

    public Shooter()
    {
        motor= Hardware.mch1;
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void shoot()
    {
        motor.setPower(0.86);
    }

    public void pause()
    {
        motor.setPower(0);
    }
}
