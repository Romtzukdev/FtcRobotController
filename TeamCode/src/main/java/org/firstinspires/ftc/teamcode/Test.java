package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.purepursuit.Path;
import com.arcrobotics.ftclib.purepursuit.Waypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.EndWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.GeneralWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.StartWaypoint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous
public class Test extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Waypoint wp = new StartWaypoint(0,0);
        Waypoint wp2 = new GeneralWaypoint(10,10);
        Waypoint wp3 = new EndWaypoint();
        DcMotor frontLeftDrive;
        DcMotor backLeftDrive;
        DcMotor frontRightDrive;
        DcMotor backRightDrive;
        frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_drive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "back_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");

        frontLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        MecanumDrive drive = new MecanumDrive(hardwareMap);
        Path p = new Path(wp,wp2,wp3);
        p.init();
        waitForStart();
        p.followPath()
    }
}
