/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Silver Material Side Autonomous", group="Linear Opmode")
//@Disabled
public class FTCAutonomousSilver extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;

    private DcMotor lift = null;

    private Servo deployer = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDrive  = hardwareMap.get(DcMotor.class, "left_motor");
        rightDrive = hardwareMap.get(DcMotor.class, "right_motor");

        lift = hardwareMap.get(DcMotor.class, "lift");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        //Motors might need to be put in brake mode set that here
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //instatiate servo(s)
        deployer = hardwareMap.get(Servo.class, "deployer");
        deployer.setDirection(Servo.Direction.FORWARD);
        deployer.setPosition(1);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        int part = 1;
        int originalLiftPosition = 0;
        double startRuntime = 0;

        PathfinderDrive pathfinderDrive = null;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double leftPower;
            double rightPower;

            switch(part){

                //read original lift encoder position
                case 1:
                    originalLiftPosition = lift.getCurrentPosition();
                    part = 10;
                    break;

                //move the lift and in the process lower the robot to the ground
                case 10:

                    lift.setPower(.5);
                    if(((lift.getCurrentPosition() - originalLiftPosition) / 1013) >= 6.5){
                        lift.setPower(0);
                        part = 20;
                    }
                    break;

                //setup the drive from lander to depot

                case 20:
                    startRuntime = runtime.seconds();
                    part = 21;
                    break;

                case 21:
                    rightDrive.setPower(0.5);
                    if(runtime.seconds() - startRuntime >= 1.5){
                        rightDrive.setPower(0);
                        part = 22;
                    }
                    break;

                case 22:

                    pathfinderDrive = new PathfinderDrive(telemetry, hardwareMap.appContext, leftDrive, rightDrive, R.raw.lefttrajectorystep1traj1, R.raw.righttrajectorystep1traj1);
                    pathfinderDrive.setup(false);
                    part = 30;
                    break;

                case 30:

                    leftDrive.setPower(pathfinderDrive.leftEncoderFollower.calculate(leftDrive.getCurrentPosition()));
                    rightDrive.setPower(pathfinderDrive.rightEncoderFollower.calculate(rightDrive.getCurrentPosition()));
                    if(pathfinderDrive.leftEncoderFollower.isFinished()){
                        leftDrive.setPower(0);
                        rightDrive.setPower(0);
                        part = 40;
                    }
                    break;

                //set the team marker down
                case 40:

                    deployer.setPosition(0);
                    startRuntime = runtime.seconds();
                    part = 50;
                    break;

                //wait for a time, allowing the marker to fall off
                case 50:

                    if(runtime.seconds() - startRuntime >= 2){
                        deployer.setPosition(1);
                        part = 60;
                    }
                    break;

                case 60:

                    pathfinderDrive = new PathfinderDrive(telemetry, hardwareMap.appContext, leftDrive, rightDrive, R.raw.lefttrajectorystep2traj1, R.raw.righttrajectorystep2traj1);
                    pathfinderDrive.setup(true);
                    part = 70;
                    break;

                case 70:

                    leftDrive.setPower(-pathfinderDrive.leftEncoderFollower.calculate(leftDrive.getCurrentPosition()));
                    rightDrive.setPower(-pathfinderDrive.rightEncoderFollower.calculate(rightDrive.getCurrentPosition()));
                    if(pathfinderDrive.leftEncoderFollower.isFinished()){
                        leftDrive.setPower(0);
                        rightDrive.setPower(0);
                        part = 80;
                    }
                    break;

                case 80:

                    break;

            }



            // Show the elapsed game time
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
}
