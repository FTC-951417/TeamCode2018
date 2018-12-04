package org.firstinspires.ftc.teamcode;

import android.content.Context;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by team10058 on 11/9/2018.
 */

public class PathfinderDrive {

    Context context = null;
    Telemetry telemetry = null;

    DcMotor left = null;
    DcMotor right = null;

    public EncoderFollower leftEncoderFollower = null;
    public EncoderFollower rightEncoderFollower = null;

    int leftFile = 0;
    int rightFile = 0;

    public PathfinderDrive(Telemetry telemetry, Context context, DcMotor left, DcMotor right, int leftFile, int rightFile){

        this.telemetry = telemetry;
        this.context = context;
        this.left = left;
        this.right = right;
        this.leftFile = leftFile;
        this.rightFile = rightFile;

    }

    public void setup(boolean backward){

        Reader reader = new Reader();
        Segment[] leftSide = reader.getSegments(leftFile, telemetry, context);
        Segment[] rightSide = reader.getSegments(rightFile, telemetry, context);
        leftEncoderFollower = new EncoderFollower(leftSide);
        rightEncoderFollower = new EncoderFollower(rightSide);
        if(!backward) {
            leftEncoderFollower.configureEncoder(left.getCurrentPosition(), 1120, 3.369);
            leftEncoderFollower.configurePIDVA(1, 0.0, 0.0, 1 / 15, 0.0);
            rightEncoderFollower.configureEncoder(right.getCurrentPosition(), 1120, 3.369);
            rightEncoderFollower.configurePIDVA(1, 0.0, 0.0, 1 / 15, 0.0);
        }else{
            leftEncoderFollower.configureEncoder(left.getCurrentPosition(), -1120, 3.369);
            leftEncoderFollower.configurePIDVA(1, 0.0, 0.0, 1 / 15, 0.0);
            rightEncoderFollower.configureEncoder(right.getCurrentPosition(), -1120, 3.369);
            rightEncoderFollower.configurePIDVA(1, 0.0, 0.0, 1 / 15, 0.0);
        }

    }

}
