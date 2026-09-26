package org.firstinspires.ftc.teamcode.chassis;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.util.SubsystemIF;

public class Chassis extends SubsystemIF {
    private Telemetry telemetry;
    private HardwareMap hardwareMap;
    private Follower follower;

    public Chassis(Telemetry telemetry, HardwareMap hardwareMap) {
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
    }


    @Override
    public void autonomousInit() {
        follower = Constants.createAutonomous(hardwareMap);
    }

    @Override
    public void teleopInit() {
        follower = Constants.createTeleop(hardwareMap);
        resetHeading();
    }

    public Pose getPose() {
        return follower.pose();
    }

    public boolean isFollowingPath() {
        return follower.following();
    }

    public boolean isPathFinished() {
        return !follower.isBusy() || !(isFollowingPath() || follower.holding());
    }

    public void followPath(Path path) {
        follower.follow(path);
    }

    public void setPose(Pose pose) {
        follower.setPose(pose);
    }

    public void stopFollower() {
        follower.stop();
    }

    public void setHoldEnd(boolean holdEnd) {
        follower.holdEnd.set(holdEnd);

    }

    public void holdCurrentPose() {
        follower.hold(getPose());
    }

    public void resetHeading() {
            follower.setHeading(0);
    }

    public void setDrivePowers(double fwd, double str, double rot) {
        DrivePowers powers = ManualDrive.fieldCentric(
                fwd, str, rot, follower.pose().heading());
        follower.manual(powers);
    }


    @Override
    public void periodic() {
        follower.update();

        telemetry.addLine();
        telemetry.addData("Is Following Path", isFollowingPath());
    }
}
