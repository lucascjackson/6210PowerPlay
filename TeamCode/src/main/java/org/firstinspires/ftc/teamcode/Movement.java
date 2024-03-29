package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.stellaris.FlashLoaderManager;

public class Movement {

    HardwareMap robot;
    private DcMotor FL;
    private DcMotor FR;
    private DcMotor BL;
    private DcMotor BR;


    public Movement(HardwareMap hardwareMap) {
        this.robot = hardwareMap;

        FL = robot.get(DcMotor.class, "FL");
        FR = robot.get(DcMotor.class, "FR");
        BL = robot.get(DcMotor.class, "BL");
        BR = robot.get(DcMotor.class, "BR");

        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        FL.setDirection(DcMotor.Direction.FORWARD);
        FR.setDirection(DcMotor.Direction.REVERSE);
        BL.setDirection(DcMotor.Direction.REVERSE);
        BR.setDirection(DcMotor.Direction.REVERSE);

    }

    public double[] holonomicDrive(double leftX, double leftY, double rightX) {
        double[] motorpowers = new double[4];
        motorpowers[0] = leftY - leftX - rightX;
        motorpowers[1] = leftY + leftX + rightX;
        motorpowers[2] = leftY + leftX - rightX;
        motorpowers[3] = leftY - leftX + rightX;
        return motorpowers;
    }

    public void setPowers(double[] motorPower) {

        FR.setPower(motorPower[0]);
        FL.setPower(motorPower[1]);
        BR.setPower(motorPower[2]);
        BL.setPower(motorPower[3]);
    }

    public void setPowers(double fr, double fl, double br, double bl) {

        FR.setPower(fr);
        FL.setPower(fl);
        BR.setPower(br);
        BL.setPower(bl);
    }

    public void AML1Park(int colorCase) {

        double[] forward = {0.5, 0.5, 0.5, 0.5};
        double[] backward = {-0.5, -0.5, -0.5, -0.5};
        double[] stop = {0,0,0,0};
        double[] turn = {0.5, -0.5, 0.5, -0.5};

        ElapsedTime time = new ElapsedTime();

        time.reset();
        time.startTime();

        setPowers(forward);

        while (time.seconds() < 1.3);

        setPowers(stop);

        time.reset();
        time.startTime();

        setPowers(turn);

        while (time.seconds() < 1);

        setPowers(stop);



        time.reset();
        time.startTime();

        switch(colorCase) {

            case 1:
                setPowers(backward);
                break;
            case 2:
                setPowers(stop);
                break;

            case 3:
                setPowers(forward);
                break;
        }


        while (time.seconds() < 1.2);

        setPowers(stop);

    }


}
