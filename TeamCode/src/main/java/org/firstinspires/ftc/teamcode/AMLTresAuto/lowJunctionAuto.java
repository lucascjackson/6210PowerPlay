package org.firstinspires.ftc.teamcode.AMLTresAuto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Manipulators;
import org.firstinspires.ftc.teamcode.Movement;
import org.firstinspires.ftc.teamcode.VuforiaBitMap3;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@Autonomous(name = "lowJunctionAuto", group = "Autonomous")

public class lowJunctionAuto extends LinearOpMode{
    enum State {

        IDLE,
        WAIT,
        POS_ON_WALL,
        GO_FORWARD,
        CYCLE_LOW,
        PARK
    }

    Pose2d startPose = new Pose2d(0, 0, Math.toRadians(180));
    Pose2d startPose2 = new Pose2d(0, 0, Math.toRadians(180));
    Pose2d startPose3 = new Pose2d(0, 0, Math.toRadians(0));

    Pose2d posEstimate;

    public static double accuracyVar = 0.5;

    public static double goToWallX = -3;
    public static double goToWallY = -70;
    public static double gotoWallAngle = Math.toRadians(180);

    public static double goToStackX =-35;
    public static double goToStackY = -70;
    public static double goToStackAngle = Math.toRadians(180);

    public static double turnToAlignX = 2;
    public static double turnToAlignY = 5;
    public static double turnToAlignAngle = 3.55;

    public static double faceWallX = 0;
    public static double faceWallY = 10;
    public static double faceWallAngle = 5.0;

    public static double pickUpConeX = 0;
    public static double pickUpConeY = 21;
    public static double pickUpConeAngle = 5.29;

    public static double moveBack = 15 ;

    public static double scoreLowX = 20 ;
    public static double scoreLowY = 5;
    public static double scoreLowAngle = -2.65;

    public static double parkUnoX = 28;
    public static double ParkUnoY = -20;
    public static double parkUnoAngle = -2.65;

    public static double parkDosX = 28;
    public static double parkDosY = 20;
    public static double parkDosAngle = -2.52;

    public static double parkTresX = 28;
    public static double parkTresY = 20;
    public static double parkTresAngle = -2.52;

    public static double startWait = 0;

    int cycleCount;

    lowJunctionAuto.State currentState = lowJunctionAuto.State.IDLE;

    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        VuforiaBitMap3 vuforia = new VuforiaBitMap3(this);
        Movement move = new Movement(hardwareMap);
        Manipulators manip = new Manipulators(hardwareMap);

        int pos = (int) vuforia.leftPositionVision();
        telemetry.addData("Pos: ", pos);
        telemetry.update();

        drive.setPoseEstimate(startPose);

        ElapsedTime waitTimer = new ElapsedTime();

        telemetry.addLine("init done");
        telemetry.update();

        waitForStart();

        TrajectorySequence posOnWall = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(goToWallX, goToWallY, gotoWallAngle))
                .lineToLinearHeading(new Pose2d(goToStackX, goToStackY, goToStackAngle))
                .build();

        Trajectory turnToAlign = drive.trajectoryBuilder(startPose2)
                .lineToLinearHeading(new Pose2d(turnToAlignX, turnToAlignY, turnToAlignAngle))
                .build();

        Trajectory faceWall = drive.trajectoryBuilder(turnToAlign.end())
                .lineToLinearHeading(new Pose2d(faceWallX, faceWallY, faceWallAngle))
                .build();

        Trajectory pickUpCone = drive.trajectoryBuilder(faceWall.end())
                .lineToLinearHeading(new Pose2d(pickUpConeX, pickUpConeY, pickUpConeAngle))
                .build();

        TrajectorySequence scoreLow = drive.trajectorySequenceBuilder(startPose3)
                .forward(moveBack)
                .lineToLinearHeading(new Pose2d(scoreLowX, scoreLowY, scoreLowAngle))
                .build();

        TrajectorySequence pos_uno = drive.trajectorySequenceBuilder(scoreLow.end())
                .lineToLinearHeading(new Pose2d(parkUnoX, ParkUnoY, parkUnoAngle))
                .build();

        TrajectorySequence pos_dos = drive.trajectorySequenceBuilder(scoreLow.end())
                .lineToLinearHeading(new Pose2d(parkDosX, parkDosY, parkDosAngle))
                .build();

        TrajectorySequence pos_tres = drive.trajectorySequenceBuilder(scoreLow.end())
                .lineToLinearHeading(new Pose2d(parkTresX, parkTresY, parkTresAngle))
                .build();

        currentState = lowJunctionAuto.State.WAIT;



        while (opModeIsActive() && !isStopRequested()) {

            posEstimate = drive.getPoseEstimate();

            switch (currentState) {

                case WAIT:

                    if (waitTimer.seconds() > startWait) {

                        currentState = State.POS_ON_WALL;
                    }

                    break;

                case POS_ON_WALL:

                    drive.followTrajectorySequence(posOnWall);

                    manip.clawClose();

                    drive.followTrajectory(turnToAlign);


                    while (!manip.colorIsActive()) {
                        move.setPowers(0.375,0.3,0.3,0.34999995);
                    }

                    move.setPowers(0,0,0,0);
                    manip.moveLift(-800);
                    manip.clawOpen();

                    drive.followTrajectory(faceWall);

                    currentState = State.GO_FORWARD;

                    break;

                case GO_FORWARD:

                    drive.followTrajectory(pickUpCone);

                    manip.clawClose();

                    sleep(800);

                    currentState = State.CYCLE_LOW;

                    break;

                case CYCLE_LOW:

                    manip.moveLift(-1000);

                    drive.followTrajectorySequence(scoreLow);

                    manip.clawOpen();
                    sleep(500);

                    currentState = State.PARK;

                    break;

                case PARK:
                    if (pos == 1) {
                        drive.followTrajectorySequence(pos_uno);
                        currentState = lowJunctionAuto.State.IDLE;
                    }
                    else if (pos == 2){
                        drive.followTrajectorySequence(pos_dos);
                        currentState = lowJunctionAuto.State.IDLE;
                    }
                    else{
                        drive.followTrajectorySequence(pos_tres);
                        currentState = lowJunctionAuto.State.IDLE;
                    }

                    break;

                case IDLE:

                    break;
            }
        }
    }
}
