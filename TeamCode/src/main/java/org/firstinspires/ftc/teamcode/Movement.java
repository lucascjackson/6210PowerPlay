package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Movement {

    HardwareMap robot;
    private DcMotor FL;
    private DcMotor FR;
    private DcMotor BL;
    private DcMotor BR;
    private boolean halfspeed = false;

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
        motorpowers[0] = leftY + leftX + rightX;
        motorpowers[1] = leftY - leftX - rightX;
        motorpowers[2] = leftY - leftX + rightX;
        motorpowers[3] = leftY + leftX - rightX;
        return motorpowers;
    }

    public void setPowers(double[] motorPower) {

        if (halfspeed) {

            for(int i = 0; i<4; i++) {
                motorPower[i] /= 2;
            }
        }

        FR.setPower(motorPower[0]);
        FL.setPower(motorPower[1]);
        BR.setPower(motorPower[2]);
        BL.setPower(motorPower[3]);
    }

    public void setHalfspeed() {

        halfspeed = !halfspeed;

    }


    public void AML1Park(int colorCase) {

        double[] forward = {0.5, 0.5, 0.5, 0.5};
        double[] stop = {0,0,0,0};

        ElapsedTime time = new ElapsedTime();

        time.reset();

        setPowers(forward);

        while (time.seconds() < 1);

        setPowers(stop);

        double[] strafe = new double[4];

        switch(colorCase) {
            case 1:
                strafe[0] = -0.5;
                strafe[1] = 0.5;
                strafe[2] = -0.5;
                strafe[3] = 0.5;

            case 2:
                strafe[0] = 0.5;
                strafe[1] = -0.5;
                strafe[2] = 0.5;
                strafe[3] = -0.5;

            case 3:
                strafe[0] = 0;
                strafe[1] = 0;
                strafe[2] = 0;
                strafe[3] = 0;
        }

    }


}
