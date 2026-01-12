package org.firstinspires.ftc.teamcode.Modules.Intake;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Robot.Hardware;

@Config
public class Intake {

    public enum State{
        INTAKE, REVERSE , REPAUS;
    }State state;

    public static double intakePower=1 , reversePower=-1;
    public static boolean REVERSE=true;
    DcMotorEx motor;
    public Intake()
    {
        state=State.REPAUS;
        motor= Hardware.mch0;
        if(REVERSE)motor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setState(State state)
    {
        this.state=state;
    }

    private void updateHardware()
    {
        switch (state)
        {
            case INTAKE:
                motor.setPower(intakePower);
                break;
            case REVERSE:
                motor.setPower(reversePower);
                break;
            case REPAUS:
                motor.setPower(0);
                break;
        }
    }

    public void update()
    {
        updateHardware();
    }
}
