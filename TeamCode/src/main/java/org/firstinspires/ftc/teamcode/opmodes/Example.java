package org.firstinspires.ftc.teamcode.opmodes;

//import all of the classes we need from the ftc sdk, we will be doing this for each bit of hardware that we use
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

//Specify that this is teleoperated opmode, and name it "example opmode." THe Linear Opmode is just telling it to show up in the driver station
@TeleOp(name="Example Opmode", group="Linear Opmode")
public class Example extends LinearOpMode {

    //Put variables here that we want to not reset every loop
    double motorPower = 0;

    //this part of code runs after we initialize the opmode, but have not started it yet
    //note the difference between "initializing" an opmode vs "starting" it
    public void runOpMode() {
        //show on the driver station screen that we have initialized the opmode
        telemetry.addData("Status", "Initialized");

        //create a new motor that we can power and move later
        DcMotor motor = hardwareMap.dcMotor.get("motor");

        //wait until the opmode has been started
        //this ensures that we are not running the above code over and over again until it as started
        //this is because all the things here have already been initialized, and do not need to be again
        waitForStart();
        //this code will run over and over again once the opmode has been started in a loop
        //This code will stop once the opmode is ended
        while (opModeIsActive()) {

            //set the value of the "motorPower" variable to the value of the joystick/ a number
            motorPower = 1;
            //motorPower = gamepad1.right_stick_y;

            //set the motor power to the value of the variable "motorPower"
            motor.setPower(motorPower);

            //add data on the driver station screen which says what the value of "motorPower" variable is, and thus what the speed of the motor is right now
            telemetry.addData("motorPower: ", motorPower);
            // add telemetry of what the joystick power is at
            telemetry.addData("joystick: ", gamepad1.right_stick_y);
            //update the telemetry every loop so that it is up to date with the real values
            telemetry.update();
        }

    }
}
