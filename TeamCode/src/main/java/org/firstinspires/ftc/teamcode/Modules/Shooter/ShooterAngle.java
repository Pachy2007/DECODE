package org.firstinspires.ftc.teamcode.Modules.Shooter;

import org.firstinspires.ftc.teamcode.Modules.Drive.Odo;
import org.firstinspires.ftc.teamcode.Robot.Hardware;
import org.firstinspires.ftc.teamcode.Wrappers.BetterMotor;
import org.firstinspires.ftc.teamcode.Wrappers.BetterServo;

public class ShooterAngle {


    public static double farest=0.96 , nearest;

    BetterServo servo;

    public ShooterAngle()
    {
        servo=new BetterServo("SERVO" , Hardware.sch0, BetterServo.RunMode.GoToPosition , farest, false);
    }


    public void update()
    {
            servo.setPosition(farest);
    }
}
