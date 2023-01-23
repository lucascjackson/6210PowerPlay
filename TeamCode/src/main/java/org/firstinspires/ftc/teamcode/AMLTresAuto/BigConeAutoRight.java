package org.firstinspires.ftc.teamcode.AMLTresAuto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Manipulators;
import org.firstinspires.ftc.teamcode.VuforiaBitMap;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
@Autonomous(name = "BigConeAutoRight", group = "Autonomous")

public class BigConeAutoRight extends LinearOpMode {

    Manipulators manip;
    private DcMotor lift2;
    public DigitalChannel liftSensor;

    enum State {

        IDLE,
        WAIT,
        PUSH_CONE_AND_PICK_UP_CONE,
        GRAB_CONE_AND_DEPOSIT_CONE,
        PARK
    }

    Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
    Pose2d posEstimate;

    public static double pushConeX = 70;
    public static double pushConeY = 0;
    public static double pushConeAngle = 0;

    public static double lineToConeX = 54;
    public static double lineToConeY = 0;
    public static double lineToConeAngle = 0;

    public static double turnToConeX = 54;
    public static double turnToConeY = 6.50521;
    public static double turnToConeAngle = Math.toRadians(270);

    public static double pickUpConeX = 59;
    public static double pickUpConeY = 25;
    public static double pickUpConeAngle = Math.toRadians(270);

    public static double lineToPoleX = 59;
    public static double lineToPoleY = -20;
    public static double lineToPoleAngle = Math.toRadians(270);

    public static double turnToPoleX = 44;
    public static double turnToPoleY = -16;
    public static double turnToPoleAngle = Math.toRadians(270);

    public static double moveToDepositX = 50;
    public static double moveToDepositY = 0;
    public static double moveToDepositAngle = Math.toRadians(200);

    public static double turnToParkX = 50.001;
    public static double turnToParkY = -10.001;
    public static double turnToParkAngle = Math.toRadians(250);

    public static double startWait = 0;

    State currentState = State.IDLE;

    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        VuforiaBitMap vuforia = new VuforiaBitMap(this);

        //Lift
        lift2 = hardwareMap.dcMotor.get("lift2");
        lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftSensor = hardwareMap.get(DigitalChannel.class, "liftSensor");

        telemetry.setAutoClear(false);
        Telemetry.Item liftPosition = telemetry.addData("Lift Position", lift2.getCurrentPosition());


        while(!isStarted()){
            liftPosition.setValue(lift2.getCurrentPosition());
            telemetry.update();
        }


        int liftTarget = 0;
        double liftSpeed = 0;
        String liftDirection = "up";
        manip = new Manipulators(hardwareMap);
        drive.setPoseEstimate(startPose);

        ElapsedTime waitTimer = new ElapsedTime();

        telemetry.addLine("init done");
        telemetry.update();

        waitForStart();

        Trajectory pushCone = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(pushConeX, pushConeY,pushConeAngle ))
                .build();
        telemetry.addLine("T1");
        telemetry.update();
        Trajectory lineToCone = drive.trajectoryBuilder(pushCone.end())
                .lineToLinearHeading(new Pose2d(lineToConeX, lineToConeY, lineToConeAngle))
                .build();
        telemetry.addLine("T2");
        telemetry.update();
        Trajectory turnToCone = drive.trajectoryBuilder(lineToCone.end())
                .lineToLinearHeading(new Pose2d(turnToConeX, turnToConeY, turnToConeAngle))
                .build();
        telemetry.addLine("T3");
        telemetry.update();
        Trajectory pickUpCone = drive.trajectoryBuilder(turnToCone.end())
                .lineToLinearHeading(new Pose2d(pickUpConeX, pickUpConeY,pickUpConeAngle ))
                .build();
        telemetry.addLine("T4");
        telemetry.update();
        Trajectory lineToPole = drive.trajectoryBuilder(pickUpCone.end())
                .lineToLinearHeading(new Pose2d(lineToPoleX, lineToPoleY, lineToPoleAngle))
                .build();
        telemetry.addLine("T5");
        telemetry.update();
        Trajectory turnToPole = drive.trajectoryBuilder(lineToPole.end())
                .lineToLinearHeading(new Pose2d(turnToPoleX, turnToPoleY, turnToPoleAngle))
                .build();
        telemetry.addLine("T6");
        telemetry.update();
        Trajectory moveToDeposit = drive.trajectoryBuilder(turnToPole.end())
                .lineToLinearHeading(new Pose2d(moveToDepositX, moveToDepositY,moveToDepositAngle))
                .build();
        telemetry.addLine("T7");
        telemetry.update();
        Trajectory turnToPark = drive.trajectoryBuilder(moveToDeposit.end())
                .lineToLinearHeading(new Pose2d(turnToParkX, turnToParkY, turnToParkAngle))
                .build();
        telemetry.addLine("T8");
        telemetry.update();

        currentState = State.WAIT;
        telemetry.addData("trajectories", "built");
        waitTimer.reset();

        int pos = vuforia.LeftPostionVision();
        telemetry.addData("Pos: ", pos);
        telemetry.update();
        while (opModeIsActive() && !isStopRequested()) {

            posEstimate = drive.getPoseEstimate();

            switch (currentState) {

                case WAIT:

                    if (waitTimer.seconds() > startWait) {

                        currentState = State.PUSH_CONE_AND_PICK_UP_CONE;
                    }

                    break;

                case PUSH_CONE_AND_PICK_UP_CONE:

                    drive.followTrajectory(pushCone);
                    manip.clawClose();
                    drive.followTrajectory(lineToCone);

                    liftTarget = 700;
                    liftSpeed = 0.98;
                    liftDirection = "up";
                    lift2.setPower(liftSpeed);
                    lift2.setTargetPosition(liftTarget);
                    lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    manip.clawOpen();

                    drive.followTrajectory(turnToCone);
                    drive.followTrajectory(pickUpCone);

                    manip.clawClose();
                    //wait 1 second
                    sleep(1000);
                    liftTarget = 1900;
                    liftSpeed = 0.98;
                    liftDirection = "up";
                    lift2.setPower(liftSpeed);
                    lift2.setTargetPosition(liftTarget);
                    lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    currentState = State.GRAB_CONE_AND_DEPOSIT_CONE;


                    break;

                case GRAB_CONE_AND_DEPOSIT_CONE:
                    drive.followTrajectory(lineToPole);
                    drive.followTrajectory(turnToPole);
                    drive.followTrajectory(moveToDeposit);

                    currentState = State.PARK;


                    break;

                case PARK:
                    drive.followTrajectory(turnToPark);

                    currentState = State.IDLE;

                    break;

                case IDLE:

                    break;
            }
        }
    }
}