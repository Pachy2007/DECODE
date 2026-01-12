package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Modules.Deposit.ActiveLatch;
import org.firstinspires.ftc.teamcode.Modules.Deposit.DepositMotor;
import org.firstinspires.ftc.teamcode.Modules.Drive.MecanumDriveTrain;

import org.firstinspires.ftc.teamcode.Modules.Drive.Odo;
import org.firstinspires.ftc.teamcode.Modules.Intake.Intake;
import org.firstinspires.ftc.teamcode.Modules.Shooter.Shooter;
import org.firstinspires.ftc.teamcode.Modules.Shooter.ShooterAngle;
import org.firstinspires.ftc.teamcode.Modules.Shooter.Turret;
import org.firstinspires.ftc.teamcode.Robot.Hardware;
import org.firstinspires.ftc.teamcode.Robot.Node;
import org.firstinspires.ftc.teamcode.Wrappers.Pose2D;
import org.opencv.core.Mat;


@Config
public class AutoBlueNodes {

    public static Pose2D[] shootPosition = {
            new Pose2D(-1400 ,-100 , Math.PI/2) ,
    };

    public static Pose2D[] beforeIntakePosition = {
            new Pose2D(-1270 ,0 , Math.PI/2) ,
            new Pose2D(-1880 ,0 , Math.PI/2) ,
            new Pose2D(-2450 ,0 , Math.PI/2)
    };

    public static Pose2D[] intakePosition = {
            new Pose2D(-1270 ,1000 , Math.PI/2) ,
            new Pose2D(-1880 ,1200 , Math.PI/2) ,
            new Pose2D(-2450 ,1200 , Math.PI/2) ,

    };

    public static Pose2D[] openGatePosition={
            new Pose2D(-1400 , 1000 , Math.PI)
    };

    public static Pose2D parkPosition= new Pose2D(-1800 , 0 , Math.PI/2);

    public static double targetX=-30 , targetY=1360;

    MecanumDriveTrain driveTrain;
    Intake intake;
    ActiveLatch latch;
    DepositMotor deposit;
    Turret turret;
    Shooter shooter;
    ShooterAngle shooterAngle;


    ElapsedTime shootingTime=new ElapsedTime();

    Node beforeShoot, shoot, beforeIntake, intaking, openGate, park;
    public Node currentNode;

    public void init(HardwareMap hardwareMap)
    {
        Hardware.init(hardwareMap);

        driveTrain=new MecanumDriveTrain(MecanumDriveTrain.State.PID);
        intake=new Intake();
        latch=new ActiveLatch();
        deposit=new DepositMotor(DepositMotor.State.REPAUS);
        turret=new Turret();
        turret.reset();
        shooter=new Shooter();
        shooterAngle=new ShooterAngle();

        Turret.targetX=targetX;
        Turret.targetY=targetY;

        latch = new ActiveLatch();
        latch.setState("close");
        beforeShoot = new Node("beforeShoot");
        shoot = new Node("shoot");
        beforeIntake = new Node("beforeIntake");
        intaking = new Node("intake");
        openGate = new Node("openGate");
        park = new Node("park");



        beforeShoot.addConditions(
                ()->{
                    deposit.setState(DepositMotor.State.INTAKE);
                    latch.setState("close");
                    shooter.shoot();
                    driveTrain.setTargetPosition( shootPosition[Math.min(shootPosition.length-1 , beforeShoot.index)]);
                }
                ,
                ()->{
                    if(driveTrain.inPosition(100 , 100 , 0.1))
                    {
                        shootingTime.reset();
                        return true;
                    }
                    return false;
                }
                ,
                new Node[]{shoot}
        );

        shoot.addConditions(
                ()->{
                    intake.setState(Intake.State.REPAUS);
                    deposit.setState(DepositMotor.State.INTAKE);
                    latch.setState("open");
                }
                ,
                ()->{
                    return  shootingTime.seconds()>3.1;
                }
                ,
                new Node[]{beforeIntake , beforeIntake , beforeIntake , park}
        );

        beforeIntake.addConditions(
                ()->{
                    shooter.pause();
                    deposit.setState(DepositMotor.State.REPAUS);
                    latch.setState("close");
                    driveTrain.setTargetPosition(beforeIntakePosition[Math.min(beforeIntakePosition.length-1 , beforeIntake.index)  ]);
                }
                ,
                ()->{
                    return driveTrain.inPosition(30 , 30 , 0.1);
                }
                ,
                new Node[]{intaking}
        );
        intaking.addConditions(
                ()->{
                    intake.setState(Intake.State.INTAKE);
                    deposit.setState(DepositMotor.State.INTAKE);
                    driveTrain.setTargetPosition(intakePosition[Math.min(intakePosition.length-1 , intaking.index)  ]);
                }
                ,
                ()->{
                    if(driveTrain.inPosition(160 , 160 , 0.2) || (Math.abs(Odo.odo.getVelY())<10 && driveTrain.inPosition(200 ,200 , 0.5)))
                    {
                        return true;
                    }
                    return false;
                }
                ,
                new Node[]{openGate, beforeShoot}
        );

        openGate.addConditions(
                ()->{
                    driveTrain.setTargetPosition(openGatePosition[Math.min(openGatePosition.length-1 , openGate.index)]);
                }
                ,
                ()->{
                    return driveTrain.inPosition(80 , 130 , 0.15);
                }
                ,
                new Node[]{beforeShoot}
        );

        park.addConditions(
                ()->{
                    driveTrain.setTargetPosition(parkPosition);
                }
                ,
                ()->{
                    return false;
                }
                ,
                new Node[]{park}
        );




        currentNode=beforeShoot;
    }

    public void run(Telemetry telemetry)
    {
        currentNode.run();

        if(currentNode.transition())currentNode=currentNode.next[Math.min(currentNode.index++ , currentNode.next.length-1)];

        Odo.update();

        driveTrain.update();
        intake.update();
        latch.update();
        turret.update();
        deposit.update();
        shooterAngle.update();
    }
}