package org.firstinspires.ftc.teamcode.Modules.Deposit;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Robot.Hardware;
import org.firstinspires.ftc.teamcode.Robot.IServoModule;
import org.firstinspires.ftc.teamcode.Wrappers.BetterServo;

import java.lang.invoke.LambdaConversionException;

@Config
public class ActiveLatch extends IServoModule {


    public static boolean reverse=false;

    public static double close=0.5 , open=0.7;

    public static double MaxVelocoty=15 , Acceleration=16  , Deceleration=16;

    public ActiveLatch()
    {
        moduleName="ACTIVE_LATCH";
        this.acceleration=Acceleration;
        this.deceleration=Deceleration;
        this.maxVelocity=MaxVelocoty;
        setStates();

        setServos(
                new BetterServo("SERVO" , Hardware.seh0, BetterServo.RunMode.PROFILE, open , reverse)
        );
        setProfileCoefficients();
        atStart();


    }

    @Override
    public void setStates() {
        states.addState("close" , close);
        states.addState("open" , open);
    }

    @Override
    public void updateStatesPosition() {
        states.get("open").updatePositions(open);
        states.get("close").updatePositions(close);
    }

    @Override
    public void atStart() {
        state=states.get("open");
    }
}
