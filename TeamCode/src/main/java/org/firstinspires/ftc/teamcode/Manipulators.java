package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;

@Config
public class Manipulators {

    private HardwareMap robot = null;
    //Lowest and Highest Encoder Positions
    public int lowest = 0;
    public int highest = 0;
    //lift
    public DcMotor lift1;
    public DcMotor lift2;
    public DigitalChannel liftSensor;
    public int defaultPos;
    public int pos1;
    public int pos2;
    public int pos3;

    //claw
    public Servo claw;
    public boolean open;
    private LinearOpMode opMode;

    //color sensor
    public ColorSensor colorSensorFront;
    public ColorSensor colorSensorBack;

    //Hardware Map
    public Manipulators(HardwareMap robot) {
        this.robot = robot;
        //lift mapping
        lift1 = robot.get(DcMotor.class, "lift1");
        lift2 = robot.get(DcMotor.class, "lift2");
        liftSensor = robot.get(DigitalChannel.class, "liftSensor");

        //claw mapping
        claw = robot.get(Servo.class, "claw");

        //color sensor
        colorSensorBack = robot.colorSensor.get("colorSensorBack");
        colorSensorFront = robot.colorSensor.get("colorSensorFront");

        //lift Encoders
        lift1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lift1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lift2.setDirection(DcMotor.Direction.REVERSE);

    }

    //LIFT METHODS
    //Use encoder ticks to move the lift up 3 levels

    public void powerLift(double power) {
        //lift1.setPower(power);
        lift2.setPower(power);
        lift1.setPower(power);
    }




   /* public int getLiftPosition() {
        return lift1.getCurrentPosition();
    }*/

    public boolean liftIsDefault() {
        return !liftSensor.getState();
    }

    //CLAWS METHODS

    public void clawOpen() {
        claw.setPosition(.6);
        open = true;
    }

    public void clawClose() {
        claw.setPosition(1.1);
        open = false;
    }

    public boolean clawIsOpen() {
        return open;
    }

    public boolean colorBackIsActive() {
        if (colorSensorBack.red() > 0 || colorSensorBack.blue() > 0) {
            return true;
        } else {
            return false;
        }
    }

    public boolean colorFrontIsActive() {
        if (colorSensorFront.red() > 0 || colorSensorFront.blue() > 0) {
            return true;
        } else {
            return false;
        }
    }

    public int colorRedBack() {
        return colorSensorBack.red();
    }

    public int colorBlueBack() {
        return colorSensorBack.blue();
    }

    public int colorRedFront() {
        return colorSensorFront.red();
    }

    public int colorBlueFront() {
        return colorSensorFront.blue();
    }

}