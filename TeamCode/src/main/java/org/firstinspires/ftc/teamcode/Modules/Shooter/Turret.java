package org.firstinspires.ftc.teamcode.Modules.Shooter;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Math.PIDController;
import org.firstinspires.ftc.teamcode.Modules.Drive.Odo;
import org.firstinspires.ftc.teamcode.Robot.Hardware;
import org.firstinspires.ftc.teamcode.Wrappers.BetterMotor;
import org.opencv.core.Mat;

@Config
public class Turret {


    DcMotorEx motor;
    DcMotorEx encoder;


    public static double targetX=3000, targetY=2000;
    public static double angle;
    public static double Kp=3.2 , Ki , Kd;
    public static double position;
    public static boolean RESET=false;
    PIDController controller=new PIDController(Kp, Ki, Kd);

    public Turret()
    {
        motor= Hardware.meh0;
        encoder=Hardware.meh1;
        //encoder.setDirection(DcMotorSimple.Direction.REVERSE);
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        if(!RESET){
        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RESET=true;
        }
    }
    public void reset()
    {
        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void updateTargetAngle()
    {
        angle = Math.atan((targetY- Odo.getY())/(targetX-Odo.getX()));
        angle-=(   (Math.signum(Odo.getHeading())* Math.abs(Odo.getHeading())%(Math.PI*2)));
        if (Math.abs(angle) > Math.PI) {
            angle=-Math.signum(angle)*(Math.PI*2-Math.abs(angle));
        }
    }
    private void updateHardware()
    {
         position=encoder.getCurrentPosition()/(384.5*4)*Math.PI*2;
        double error=position-angle;

        double power=controller.calculate(0 , error);

        motor.setPower(power);
    }

    public void update()
    {
        controller.kp=Kp;
        controller.ki=Ki;
        controller.kd=Kd;
        updateTargetAngle();
        updateHardware();
    }
}
