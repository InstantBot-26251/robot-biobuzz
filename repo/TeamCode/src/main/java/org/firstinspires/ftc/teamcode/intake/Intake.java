package org.firstinspires.ftc.teamcode.intake;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.controller.PIDFController;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.SubsystemIF;

public class Intake extends SubsystemIF {
    private Telemetry telemetry;
    private HardwareMap hardwareMap;
    private DcMotorEx intakeMotor;
    private final PIDFController intakePID;

    public Intake(Telemetry telemetry, HardwareMap hardwareMap) {
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;

        intakePID = new PIDFController(IntakeConstants.INTAKE_P, IntakeConstants.INTAKE_I,
                IntakeConstants.INTAKE_D, IntakeConstants.INTAKE_F);

        this.intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
    }

    @Override
    public void autonomousInit() {
        setupMotors();
    }

    @Override
    public void teleopInit() {
        setupMotors();
    }

    public void setupMotors() {
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void intake() {
        setTargetVelocity(IntakeConstants.INTAKE_VELOCITY);
    }

    public void outtake() {
        setTargetVelocity(IntakeConstants.OUTTAKE_VELOCITY);
    }

    public void stop() {
        setTargetVelocity(IntakeConstants.STOP_INTAKE_VELOCITY);
    }

    private void setTargetVelocity(double rpm) {
        intakePID.setSetPoint(IntakeConstants.toTicksPerSec(rpm));
    }

    public double getIntakeVelocity() {
        return intakeMotor.getVelocity();
    }

    @Override
    public void periodic() {

        double velocity = getIntakeVelocity();
        double output = 0;

        if (intakePID.getSetPoint() == 0) {
            intakeMotor.setPower(0);
        } else {
            output = intakePID.calculate(velocity)
                    / IntakeConstants.toTicksPerSec(IntakeConstants.MAX_ACHIEVABLE_RPM);
            intakeMotor.setPower(output);
        }

        telemetry.addData("Intake Target Velocity", intakePID.getSetPoint());
        telemetry.addData("Intake Velocity", velocity);
        telemetry.addData("Intake PID Output", output);
    }
}
