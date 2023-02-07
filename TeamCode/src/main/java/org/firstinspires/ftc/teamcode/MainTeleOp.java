package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.configuration.annotations.MotorType;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.HashMap;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TeleOp", group="TeleOp")
public class MainTeleOp extends OpMode {

    Manipulators manip;
    Movement move;
    ElapsedTime waitTimer = new ElapsedTime();

    public HashMap<String, Boolean> buttons = new HashMap<String, Boolean>();
    double[] motorPower = {0, 0, 0, 0};
    boolean open = true;
    private int halfspeedDivider;
   // private int lift1PosDefault = manip.lift1.getCurrentPosition();
   // private int lift2PosDefault = manip.lift2.getCurrentPosition();


    String stringPos = "default";

    int[] positions;

    public void init() {
        manip = new Manipulators(hardwareMap);
        move = new Movement(hardwareMap);

        manip.clawOpen();

        //positions = manip.setPositions();
        halfspeedDivider = 1;

        telemetry.addData("init", "completed");
        telemetry.update();
    }

    public boolean isPressed(String name, boolean button){
        boolean output = false;

        //If the hashmap doesn't already contain the key
        if (!buttons.containsKey(name)) {
            buttons.put(name, false);
        }

        boolean buttonWas = buttons.get(name);
        if (button != buttonWas && button == true){
            output = true;
        }

        buttons.put(name, button);

        return output;
    }

    /*int liftTarget = 0;
    double liftSpeed = 0;
    String liftCurrentDirection = "up"*/

    @Override
    public void loop()
    {
       double leftY = 0;
       double leftX = 0;
       double rightX = 0;

       telemetry.setAutoClear(false);
       telemetry.addData("lift1 position", manip.lift1.getCurrentPosition());
      // telemetry.addData("lift2 position", manip.lift2.getCurrentPosition());
       telemetry.update();

       /*if (gamepad2.a){
           liftTarget = 200;
       liftSpeed = 0.5;
       liftCurrentDirection = "up";

       manip.lift1.setPower(liftSpeed);
       manip.lift2.setPower(liftSpeed);

       manip.lift1.setTargetPosition(liftTarget);
       manip.lift2.setTargetPosition(liftTarget);
       }
    }*/


        /*if (!manip.liftIsDefault()) {
            positions = manip.setPositions();
        }*/

        if (manip.clawIsOpen() && gamepad2.right_stick_y < -0.1)
        {
            gamepad2.rumble(0.5, 0.5, 100);
        }

        if (gamepad2.right_stick_y < -0.1 && !manip.clawIsOpen()) {
            manip.powerLift(gamepad2.right_stick_y);
        }
        else if (gamepad2.right_stick_y > 0.1 && !manip.liftIsDefault()) {
            manip.powerLift(gamepad2.right_stick_y);
        }
        else {
            manip.powerLift(0);
        }


       /* if(manip.liftIsDefault()) {
            lift1PosDefault = manip.lift1.getCurrentPosition();
            lift2PosDefault = manip.lift2.getCurrentPosition();

        }

        if(isPressed("a2", gamepad2.a)){
            manip.lift1.setTargetPosition(lift1PosDefault + 300);
            manip.lift2.setTargetPosition(lift2PosDefault + 300);

        }

        if(isPressed("b2", gamepad2.b)){
            while(!manip.liftIsDefault() || !isPressed("x2", gamepad2.x)) {

                manip.powerLift(0.5);
            }
        }*/

        if (isPressed("rightBumper2", gamepad2.right_bumper)){
            if (open){
                manip.clawClose();
                open = false;

            } else {
                manip.clawOpen();
                open = true;
            }
        }

        if (gamepad1.right_bumper) {
            halfspeedDivider = 2;
        } else {
            halfspeedDivider= 1;
        }


        if (Math.abs(gamepad1.left_stick_y)  > 0.1 ||
            Math.abs(gamepad1.left_stick_x)  > 0.1 ||
            Math.abs(gamepad1.right_stick_x) > 0.1) {

            leftY = gamepad1.left_stick_y/1.5;
            leftX = -1*(gamepad1.left_stick_x);
            rightX = (-gamepad1.right_stick_x)/1.5;

            motorPower = move.holonomicDrive(leftX/halfspeedDivider, leftY/halfspeedDivider, rightX/halfspeedDivider);

        }

        else {
            motorPower = move.holonomicDrive(0, 0, 0);
        }

        move.setPowers(motorPower[0], motorPower[1]*0.8, motorPower[2]*0.8, motorPower[3]*0.9333333333);

        //Lift MACROS

/*
         if (isPressed("2y", gamepad2.y) && stringPos.equals("default") || !stringPos.equals("high")){

            manip.liftToHeight(positions, "high");
            stringPos = "high";
        }
        else if (isPressed("2b", gamepad2.b) && stringPos.equals("default") || !stringPos.equals("mid")){

            manip.liftToHeight(positions, "mid");
            stringPos = "mid";
        }
        else if (isPressed("2a", gamepad2.a) && stringPos.equals("default") || !stringPos.equals("low")){

            manip.liftToHeight(positions, "low");
            stringPos = "low";
        }
        else if (isPressed("2a", gamepad2.a) ||
                 isPressed("2b", gamepad2.b) ||
                 isPressed("2x", gamepad2.x) ||
                 isPressed("2y", gamepad2.y)) {

            manip.liftToHeight(positions, "default");
            stringPos = "default";
        }

 */

       // telemetry.addData("Lift1 Encoder Value: ", manip.getLiftPosition());
        telemetry.addData("Lift Position: ",stringPos );
        telemetry.addData("StickY: ", gamepad2.right_stick_y);
        telemetry.addData("isInDefault: ", manip.liftIsDefault());
        telemetry.update();



        //Write a story about rishi the software guy

    }

}
