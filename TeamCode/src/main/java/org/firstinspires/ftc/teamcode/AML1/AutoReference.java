package org.firstinspires.ftc.teamcode.AML1;

/*import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Manipulators;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name = "AutoReference", group = "tesTest")

public class AutoReference extends LinearOpMode {

    enum State {
        TRAJECTORY_SET_1,
        PLACE_CONE,
        GO_TO_CONE,
        PICKUP_CONE,
        GO_TO_ROD,
        PARK,
        LIFT,
        OUTTAKE,
        IDLE
    }

    State currentState = State.IDLE;
    Pose2d startPose = new Pose2d(0,0,Math.toRadians(0));

    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDriven drive = new SampleMecanumDrive(hardwareMap);
        Manipulators manip = new Manipulators(hardwareMap);
        VuforiaBM vuforia = new VuforiaBM(this);

        drive.setPoseEstimate(startPose);

        //First trajectory approaching the cones
        Trajectory lineUp = drive.trajectoryBuilder(startPose)
            .lineToLinearHeading(new Pose2d(0, 0, Math.toRadians(0)))
            .build();

        //Wait
        double waitTime = 5;
        ElapsedTime waitTimer = new ElapsedTime();

        //Go to the cones
        Trajectory driveToCone = drive.trajectoryBuilder(lineUp.end())
            .lineToLinearHeading(new Pose2d(0, 0, Math.toRadians(0)))
            .build();

        //Pick up the cone

        //Go to junction pole
        Trajectory goToPole = drive.trajectoryBuilder(driveToCone.end())
            .lineToLinearHeading(new Pose2d(0, 0, Math.toRadians(0)))
            .build();

        //Place the cone

        //





    }

}
*/