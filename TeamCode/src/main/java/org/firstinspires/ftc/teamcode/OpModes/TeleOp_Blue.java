package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Modules.Deposit.ActiveLatch;
import org.firstinspires.ftc.teamcode.Modules.Deposit.DepositMotor;
import org.firstinspires.ftc.teamcode.Modules.Drive.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.Modules.Drive.Odo;
import org.firstinspires.ftc.teamcode.Modules.Intake.Intake;
import org.firstinspires.ftc.teamcode.Modules.Shooter.Shooter;
import org.firstinspires.ftc.teamcode.Modules.Shooter.Turret;
import org.firstinspires.ftc.teamcode.Robot.Hardware;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp_Blue extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {


        Hardware.init(hardwareMap);
        Odo.init(hardwareMap , telemetry);

        MecanumDriveTrain driveTrain = new MecanumDriveTrain(MecanumDriveTrain.State.DRIVE);
        Intake intake = new Intake();
        ActiveLatch latch = new ActiveLatch();
        DepositMotor deposit = new DepositMotor(DepositMotor.State.REPAUS);
        Turret turret = new Turret();
        Shooter shooter = new Shooter();
        Hardware.sch0.setPosition(0.94);
        waitForStart();

        while(opModeIsActive())
        {
            double X=gamepad1.left_stick_x * gamepad1.left_stick_x * Math.signum(gamepad1.left_stick_x);
            double Y=gamepad1.left_stick_y * gamepad1.left_stick_y * -Math.signum(gamepad1.left_stick_y);
            double rotation=gamepad1.right_trigger-gamepad1.left_trigger;

            double heading =-(Odo.getHeading()+Math.PI/2);

            double x=X*Math.cos(heading)-Y*Math.sin(heading);
            double y=X* Math.sin(heading)+Y*Math.cos(heading);

            if(gamepad1.left_bumper){intake.setState(Intake.State.REVERSE);}
            else if(gamepad1.right_bumper){intake.setState(Intake.State.INTAKE); deposit.setState(DepositMotor.State.INTAKE);}
            else {intake.setState(Intake.State.REPAUS);deposit.setState(DepositMotor.State.REPAUS);}

            if(gamepad1.a)
            {
                deposit.setState(DepositMotor.State.TRANSFER);
                latch.setState("open");
                shooter.shoot();
            }
            else shooter.pause();
            if(gamepad1.options)Odo.reset();
            else{latch.setState("close");}

            driveTrain.setTargetVector(x , y , -rotation);

            driveTrain.update();
            deposit.update();
            intake.update();
            latch.update();
            turret.update();
            Odo.update();

            telemetry.addData("X" , Odo.getX());
            telemetry.addData("Y" , Odo.getY());
            telemetry.addData("Heading" , Odo.getHeading());
            telemetry.addData("targetAngle" , Turret.angle);
            telemetry.addData("encoderAngle" , Turret.position);

            telemetry.addData("mch0" , Hardware.mch0.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("mch1" , Hardware.mch1.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("mch2" , Hardware.mch2.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("mch3" , Hardware.mch3.getCurrent(CurrentUnit.AMPS));

            telemetry.addData("meh0" , Hardware.meh0.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("meh1" , Hardware.meh1.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("meh2" , Hardware.meh2.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("meh3" , Hardware.meh3.getCurrent(CurrentUnit.AMPS));


            telemetry.update();

        }
    }
}
