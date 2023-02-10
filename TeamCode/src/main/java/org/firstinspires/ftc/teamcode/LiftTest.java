package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.robotcore.external.Telemetry;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Lift Test", group="TeleOp")
public class LiftTest extends LinearOpMode {
    Manipulators manip;
    private DcMotor lift2;
    private DcMotor lift1;
    public DigitalChannel liftSensor;

    @Override
    public void runOpMode() throws InterruptedException
    {
        lift2 = hardwareMap.dcMotor.get("lift2");
        lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift2.setDirection(DcMotor.Direction.REVERSE);
        lift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lift1 = hardwareMap.dcMotor.get("lift1");
        lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift1.setDirection(DcMotor.Direction.REVERSE);
        lift1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        liftSensor = hardwareMap.get(DigitalChannel.class, "liftSensor");

        telemetry.setAutoClear(false);
        Telemetry.Item lift1Position = telemetry.addData("Lift1 Position", lift1.getCurrentPosition());
        Telemetry.Item lift2Position = telemetry.addData("Lift2 Position", lift2.getCurrentPosition());


        while(!isStarted()){
            lift1Position.setValue(lift1.getCurrentPosition());
            lift2Position.setValue(lift2.getCurrentPosition());
            telemetry.update();
        }



        int liftTarget = 0;
        double liftSpeed = 0;
        String liftDirection = "up";

        while (opModeIsActive()) {

            if (gamepad1.b){ // lift UP
                liftTarget = 4300;
                liftSpeed = 0.98;
                liftDirection = "up";

                lift2.setPower(liftSpeed);
                lift2.setTargetPosition(liftTarget);
                lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            } else if (gamepad1.a){ // lift DOWN
                liftTarget = 0;
                liftSpeed = -0.98;
                liftDirection = "down";

                lift2.setPower(liftSpeed);
                lift2.setTargetPosition(liftTarget);
                lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            }

            if ( liftDirection == "down" && !liftSensor.getState()){
                liftSpeed = 0;
                lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }



            idle();
            if( lift2.isBusy() ){
                lift1Position.setValue(lift2.getCurrentPosition());
                lift2Position.setValue(lift2.getCurrentPosition());
                telemetry.update();
            }

        }
    }
}