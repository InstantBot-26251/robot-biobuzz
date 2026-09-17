package org.firstinspires.ftc.teamcode.chassis;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.qualcomm.robotcore.hardware.HardwareMap;
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
    }

    public void setDrivePowers(double fwd, double str, double rot) {
        DrivePowers powers = ManualDrive.fieldCentric(
                fwd, str, rot, follower.pose().heading());
        follower.manual(powers);
    }


    @Override
    public void periodic() {
        follower.update();
    }
}
