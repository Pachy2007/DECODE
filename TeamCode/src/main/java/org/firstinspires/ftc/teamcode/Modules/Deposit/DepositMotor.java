package org.firstinspires.ftc.teamcode.Modules.Deposit;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Robot.Hardware;
import org.firstinspires.ftc.teamcode.Wrappers.BetterMotor;

@Config
public class DepositMotor {

    public static double intakePower=1, transferPower=0.8, reversePower=-1;

    public enum State{
        INTAKE(intakePower), TRANSFER(transferPower), REVERSE(reversePower), REPAUS(0);

        double power;
        State(double power)
        {
            this.power=power;
        }

    }
    State state;
    BetterMotor motor;

    public static boolean reverse=true;

    public DepositMotor(State initialState)
    {
        state=initialState;
        motor=new BetterMotor(Hardware.meh1, BetterMotor.RunMode.RUN, reverse);
    }

    public void setState(State state)
    {
        this.state=state;
    }

    private void updateHardware()
    {
        motor.setPower(state.power);
    }

    public void update()
    {
        updateHardware();
    }
}
