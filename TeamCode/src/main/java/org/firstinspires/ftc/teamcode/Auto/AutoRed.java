package org.firstinspires.ftc.teamcode.Auto;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Modules.Drive.Odo;
import org.firstinspires.ftc.teamcode.Robot.Hardware;

@Autonomous
public class AutoRed extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        Hardware.init(hardwareMap);
        Odo.reset();
        AutoRedNodes nodes=new AutoRedNodes();

        nodes.init(hardwareMap);
        while(opModeInInit())
        {
            nodes.turret.update();
            nodes.latch.update();
        }
        waitForStart();
        while(opModeIsActive())
        {
            nodes.run(telemetry);

            telemetry.addData("X" , Odo.getX());
            telemetry.addData("Y" , Odo.getY());
            telemetry.addData("HEADING" , Odo.getHeading());
            telemetry.update();
        }
    }
}
