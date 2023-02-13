package org.firstinspires.ftc.teamcode.AMLTresAuto;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import android.text.method.MovementMethod;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.AMLOneAutos.BetterOdomPark;
import org.firstinspires.ftc.teamcode.Manipulators;
import org.firstinspires.ftc.teamcode.Movement;
import org.firstinspires.ftc.teamcode.VuforiaBitMap;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@Autonomous(name = "lowJunctionAuto", group = "Autonomous")

public class lowJunctionAuto extends LinearOpMode{
    enum State {

        IDLE,
        WAIT,
        GO_TO_WALL,
        GO_TO_STACK,
        TURN_TO_ALIGN_STACK,
        GO_FORWARD_PICK_UP_CONE,
        SCORE_LOW,
        PARK
    }

    Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
    Pose2d posEstimate;

    public static double goToWallX = 0;
    public static double goToWallY = -100;
    public static double goToWallAngle = 0;

    public static double goToStackX =-40;
    public static double goToStackY = -100;
    public static double goToStackAngle = 0;

    public static double turnToAlignX = -41;
    public static double turnToAlignY = -100;
    public static double turnToAlignAngle = 5.68;

    public static double pickUpConeX = 0;
    public static double pickUpConeY = -0;
    public static double pickUpConeAngle = Math.toRadians(0);

    public static double faceWallX = 0;
    public static double faceWallY = 0;
    public static double faceWallAngle = Math.toRadians(0);

    public static double scoreLowX = 0;
    public static double scoreLowY = 0;
    public static double scoreLowAngle = Math.toRadians(0);

    public static double turnFromScoreX = 0;
    public static double turnFromScoreY = 0;
    public static double turnFromScoreAngle = Math.toRadians(0);

    public static double parkUnoX = 0;
    public static double parkUnoY = 0;
    public static double parkUnoAngle = Math.toRadians(0);

    public static double parkDosX = 0;
    public static double parkDosY = 0;
    public static double parkDosAngle = Math.toRadians(0);

    public static double parkTresX = 0;
    public static double parkTresY = 0;
    public static double parkTresAngle = Math.toRadians(0);

    public static double startWait = 0;

    int cycleCount;

    lowJunctionAuto.State currentState = lowJunctionAuto.State.IDLE;

    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        VuforiaBitMap vuforia = new VuforiaBitMap(this);
        Movement move = new Movement(hardwareMap);
        Manipulators manip = new Manipulators(hardwareMap);

        int parkPos = vuforia.LeftPostionVision();
        telemetry.addData("Pos: ", parkPos);
        telemetry.update();

        drive.setPoseEstimate(startPose);

        ElapsedTime waitTimer = new ElapsedTime();

        telemetry.addLine("init done");
        telemetry.update();

        waitForStart();

        Trajectory goToWall = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(goToWallX, goToWallY,goToWallAngle ))
                .build();


        Trajectory goToStack = drive.trajectoryBuilder(goToWall.end())
                .lineToLinearHeading(new Pose2d(goToStackX, goToStackY, goToStackAngle))
                .build();


        Trajectory turnToAlign = drive.trajectoryBuilder(goToStack.end())
                .lineToLinearHeading(new Pose2d(turnToAlignX, turnToAlignY, turnToAlignAngle))
                .build();

        Trajectory faceWall = drive.trajectoryBuilder(turnToAlign.end())
                .lineToLinearHeading(new Pose2d(faceWallX, faceWallY, faceWallAngle))
                .build();

        Trajectory pickUpCone = drive.trajectoryBuilder(faceWall.end())
                .lineToLinearHeading(new Pose2d(pickUpConeX, pickUpConeY, pickUpConeAngle))
                .build();

        Trajectory scoreLow = drive.trajectoryBuilder(pickUpCone.end())
                .lineToLinearHeading(new Pose2d(scoreLowX, scoreLowY, scoreLowAngle))
                .build();

        Trajectory turnFromScore = drive.trajectoryBuilder(scoreLow.end())
                .lineToLinearHeading(new Pose2d(turnFromScoreX, turnFromScoreY, turnFromScoreAngle))
                .build();

        Trajectory pos_uno = drive.trajectoryBuilder(scoreLow.end())
                .lineToLinearHeading(new Pose2d(parkUnoX, (parkUnoY), parkUnoAngle))
                .build();

        Trajectory pos_dos = drive.trajectoryBuilder(scoreLow.end())
                .lineToLinearHeading(new Pose2d(parkDosX, (parkDosY), parkDosAngle))
                .build();

        Trajectory pos_tres = drive.trajectoryBuilder(scoreLow.end())
                .lineToLinearHeading(new Pose2d(parkTresX, (parkTresY), parkTresAngle))
                .build();


        currentState = lowJunctionAuto.State.WAIT;



        while (opModeIsActive() && !isStopRequested()) {

            posEstimate = drive.getPoseEstimate();

            switch (currentState) {

                case WAIT:

                    if (waitTimer.seconds() > startWait) {

                        currentState = lowJunctionAuto.State.GO_TO_WALL;
                    }

                    break;

                case GO_TO_WALL:

                    drive.followTrajectory(goToWall);
                    currentState = State.GO_TO_STACK;


                    break;

                case GO_TO_STACK:
                    drive.followTrajectory(goToStack);
                    currentState = State.TURN_TO_ALIGN_STACK;


                    break;

                case TURN_TO_ALIGN_STACK:
                    drive.followTrajectory(turnToAlign);

                    while (!manip.colorIsActive()) {
                        move.setPowers(1,0.8,0.8,0.9333333);
                    }

                    manip.clawClose();
                    manip.moveLiftTo(300);
                    wait(500);
                    manip.clawOpen();

                    drive.followTrajectory(faceWall);

                    currentState = State.GO_FORWARD_PICK_UP_CONE;


                    break;

                case GO_FORWARD_PICK_UP_CONE:

                    if (cycleCount > 1) {
                        manip.moveLiftTo(300);
                        wait(500 - cycleCount*10);
                        manip.clawOpen();
                    }

                    drive.followTrajectory(pickUpCone);

                    manip.clawClose();

                    manip.moveLiftTo(500);

                    currentState = State.SCORE_LOW;

                    cycleCount++;

                    break;

                case SCORE_LOW:

                    drive.followTrajectory(scoreLow);

                    manip.clawOpen();

                    drive.followTrajectory(turnFromScore);

                    manip.clawClose();

                    manip.returnLiftToDefault();

                    currentState = State.GO_FORWARD_PICK_UP_CONE;

                    break;

                case PARK:
                    if (parkPos == 1){
                        drive.followTrajectory(pos_uno);
                        currentState = lowJunctionAuto.State.IDLE;
                    }
                    else if (parkPos == 2){
                        drive.followTrajectory(pos_dos);
                        currentState = lowJunctionAuto.State.IDLE;
                    }
                    else{
                        drive.followTrajectory(pos_tres);
                        currentState = lowJunctionAuto.State.IDLE;
                    }
                    break;

                case IDLE:

                    break;
            }
        }
    }
}
