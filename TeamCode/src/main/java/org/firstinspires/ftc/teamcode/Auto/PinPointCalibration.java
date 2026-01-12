package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Modules.Drive.Odo;
import org.firstinspires.ftc.teamcode.Modules.Shooter.Turret;
import org.firstinspires.ftc.teamcode.Robot.Hardware;

@Autonomous
public class
PinPointCalibration extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {

        Hardware.init(hardwareMap);
        Odo.init(hardwareMap , telemetry);
        Turret turret=new Turret();
        turret.reset();
        ElapsedTime time=new ElapsedTime();

        time.startTime();

        while(opModeInInit())
        {
            if(time.seconds()>3)return;
        }
    }
}
