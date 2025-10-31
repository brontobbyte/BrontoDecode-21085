package org.firstinspires.ftc.teamcode.Programs;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "TeleOp PID Shooter", group = "Competition")
public class PIDShooterTeleOp extends LinearOpMode {

    private DcMotorEx motorIntake;
    private DcMotorEx motorShooter; // Líder
    private DcMotorEx motorShooter2; // Seguidor

    // Motores do chassi
    private DcMotorEx motorFrontLeft, motorFrontRight, motorBackLeft, motorBackRight;

    // IMU para field oriented
    private IMU imu;
    private double headingOffset = 0;
    private boolean lastBButtonState = false;

    private CRServo servoIndexer;

    private boolean shooterAtivo = false;
    private boolean lastRightBumperState = false;

    private ElapsedTime pidTimer = new ElapsedTime();
    private ElapsedTime indexerTimer = new ElapsedTime();
    private ElapsedTime intakeTimer = new ElapsedTime();

    private boolean sequenciaAtiva = false;
    private boolean intakePosicionado = false;
    private boolean aguardandoVelocidade = false;
    private boolean indexerAtivo = false;
    private boolean indexerInicialAtivo = false;

    private final double metashooter = 2000;
    private final double kP = 0.01;
    private final double kI = 0.001;
    private final double kD = 0.0001;
    private final double kF = 0.005;
    private final double seguekP = 0.001;

    private final int ajusteshoot = 100;
    private final double indexertemp = 0.5;
    private final double tol = 300;

    private final int cicloshoot =-200;

    private double lastError = 0;
    private double integral = 0;

    @Override
    public void runOpMode() {
        initializeHardware();

        telemetry.addData("Status", "Inicializado");
        telemetry.addData("Controles", "L-Bumper: Intake | R-Bumper: Shooter Toggle");
        telemetry.addData("Chassi", "Field Oriented - B para resetar frente");
        telemetry.update();

        waitForStart();

        pidTimer.reset();
        indexerTimer.reset();
        intakeTimer.reset();

        while (opModeIsActive()) {
            controlChassiFieldOriented();

            controlIntake();
            controlShooter();
            controlSequencia();
            updateTelemetry();
        }

        stopAll();
    }

    private void initializeHardware() {
        motorFrontLeft = hardwareMap.get(DcMotorEx.class, "motor_esquerda");
        motorFrontRight = hardwareMap.get(DcMotorEx.class, "motor_direita");
        motorBackLeft = hardwareMap.get(DcMotorEx.class, "motor_esquerdatras");
        motorBackRight = hardwareMap.get(DcMotorEx.class, "motor_direitatras");

        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFrontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBackRight.setDirection(DcMotorSimple.Direction.FORWARD);

        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );
        imu.initialize(parameters);

        motorIntake = hardwareMap.get(DcMotorEx.class, "motor_intake");
        motorShooter = hardwareMap.get(DcMotorEx.class, "motor_shooter");
        motorShooter2 = hardwareMap.get(DcMotorEx.class, "motor_shooter2");

        motorIntake.setDirection(DcMotorSimple.Direction.FORWARD);
        motorShooter.setDirection(DcMotorSimple.Direction.FORWARD);
        motorShooter2.setDirection(DcMotorSimple.Direction.FORWARD);

        motorShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorShooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorIntake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorShooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        servoIndexer = hardwareMap.get(CRServo.class, "servo_indexer");

        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);
        motorIntake.setPower(0);
        motorShooter.setPower(0);
        motorShooter2.setPower(0);
        servoIndexer.setPower(0);
    }

    private void controlChassiFieldOriented() {
        boolean currentBButtonState = gamepad1.b;
        if (currentBButtonState && !lastBButtonState) {
            imu.resetYaw();
            headingOffset = getHeading();
            telemetry.addData("IMU", "Frente resetada");
        }
        lastBButtonState = currentBButtonState;

        double drive = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

        double heading = getHeading() - headingOffset;
        double headingRadians = Math.toRadians(heading);

        double temp = drive * Math.cos(headingRadians) - strafe * Math.sin(headingRadians);
        strafe = drive * Math.sin(headingRadians) + strafe * Math.cos(headingRadians);
        drive = temp;

        double denominator = Math.max(Math.abs(drive) + Math.abs(strafe) + Math.abs(turn), 1);
        double frontLeftPower = (drive + strafe + turn) / denominator;
        double frontRightPower = (drive - strafe - turn) / denominator;
        double backLeftPower = (drive - strafe + turn) / denominator;
        double backRightPower = (drive + strafe - turn) / denominator;

        motorFrontLeft.setPower(frontLeftPower);
        motorFrontRight.setPower(frontRightPower);
        motorBackLeft.setPower(backLeftPower);
        motorBackRight.setPower(backRightPower);
    }

    private double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }

    private void controlIntake() {
        if (!sequenciaAtiva && gamepad2.left_bumper) {
            motorIntake.setPower(-1.0);
        } else if (!sequenciaAtiva) {
            motorIntake.setPower(0);
        }
    }

    private void controlShooter() {
        boolean currentRightBumperState = gamepad2.right_bumper;

        if (currentRightBumperState && !lastRightBumperState) {
            if (!shooterAtivo) {
                shooterAtivo = true;
                sequenciaAtiva = true;
                intakePosicionado = false;
                aguardandoVelocidade = false;
                indexerAtivo = false;
                indexerInicialAtivo = true;

                motorIntake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorIntake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                integral = 0;
                lastError = 0;
                pidTimer.reset();
                indexerTimer.reset();
                intakeTimer.reset();

                posicionarIntake();
            } else {
                shooterAtivo = false;
                sequenciaAtiva = false;
                motorShooter.setPower(0);
                motorShooter2.setPower(0);
                motorIntake.setPower(0);
                servoIndexer.setPower(0);
            }
        }

        if (shooterAtivo) {
            applyShooterPID();
        }

        lastRightBumperState = currentRightBumperState;
    }

    private void posicionarIntake() {
        motorIntake.setTargetPosition(ajusteshoot);
        motorIntake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorIntake.setPower(0.4);

        servoIndexer.setPower(-1.0);
        indexerInicialAtivo = true;

        intakePosicionado = false;
        intakeTimer.reset();
    }

    private void controlSequencia() {
        if (!sequenciaAtiva) return;

        if (indexerInicialAtivo && !intakePosicionado) {
            if (!motorIntake.isBusy() || intakeTimer.seconds() > 2.0) {
                servoIndexer.setPower(0);
                indexerInicialAtivo = false;

                motorIntake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motorIntake.setPower(0);
                intakePosicionado = true;
                aguardandoVelocidade = true;
                telemetry.addData("sequência", "vel shooter");
            }
        }

        else if (aguardandoVelocidade && shooterAtivo) {
            double currentVelocity = motorShooter.getVelocity();
            if (Math.abs(metashooter - currentVelocity) < tol) {
                executarCicloDisparo();
                aguardandoVelocidade = false;
            }
        }

        else if (indexerAtivo) {
            if (indexerTimer.seconds() >= indexertemp) {
                servoIndexer.setPower(0);
                indexerAtivo = false;
                sleep(600);
                reiniciarCiclo();
            }
        }
    }

    private void executarCicloDisparo() {
        int targetPosition = motorIntake.getCurrentPosition() + cicloshoot;
        motorIntake.setTargetPosition(targetPosition);
        motorIntake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorIntake.setPower(1.0);

        new Thread(() -> {
            try {
                Thread.sleep(600);
                servoIndexer.setPower(1.0);
                indexerAtivo = true;
                indexerTimer.reset();
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }).start();
    }

    private void reiniciarCiclo() {
        if (sequenciaAtiva && shooterAtivo) {
            aguardandoVelocidade = true;
            telemetry.addData("Sequência", "repete");
        }
    }

    private void applyShooterPID() {
        double currentTime = pidTimer.seconds();
        pidTimer.reset();
        double deltaTime = Math.max(currentTime, 0.001);

        double currentVelocity = motorShooter.getVelocity();
        double error = metashooter - currentVelocity;

        double proportional = kP * error;
        integral += error * deltaTime;
        double integralTerm = kI * integral;
        double derivative = (error - lastError) / deltaTime;
        double derivativeTerm = kD * derivative;
        double feedForward = kF * metashooter;

        double output = proportional + integralTerm + derivativeTerm + feedForward;
        output = Math.max(0, Math.min(1, output));

        motorShooter.setPower(output);

        double followerVelocity = motorShooter2.getVelocity();
        double followerError = currentVelocity - followerVelocity;
        double followerOutput = output + (seguekP * followerError);
        followerOutput = Math.max(0, Math.min(1, followerOutput));

        motorShooter2.setPower(followerOutput);
        lastError = error;
    }

    private void updateTelemetry() {
        telemetry.addData("=== STATUS ===", "");
        telemetry.addData("Intake Manual", gamepad2.left_bumper ? "On" : "Off");
        telemetry.addData("Shooter", shooterAtivo ? "On pid" : "Off");
        telemetry.addData("Sequência Ativa", sequenciaAtiva ? "s" : "n");

        if (shooterAtivo) {
            telemetry.addData("Velocidade Líder", "%.0f RPM", motorShooter.getVelocity());
            telemetry.addData("Velocidade Seguidor", "%.0f RPM", motorShooter2.getVelocity());
            telemetry.addData("Target Velocity", "%.0f RPM", metashooter);
            telemetry.addData("Erro Líder", "%.0f RPM", metashooter - motorShooter.getVelocity());

            if (sequenciaAtiva) {
                telemetry.addData("sequência", "");
                telemetry.addData("Intake pos", intakePosicionado ? "ss" : "n");
                telemetry.addData("vel shooter?", aguardandoVelocidade ? "ss" : "n");
                telemetry.addData("indexer on", indexerAtivo ? "ss" : "n");
                telemetry.addData("indexer inicial", indexerInicialAtivo ? "ss" : "n");
                telemetry.addData("pos intake", motorIntake.getCurrentPosition());
            }
        }

        telemetry.addData("botão", "");
        telemetry.addData("Left Bumper", "intakando");
        telemetry.addData("Right Bumper", "chutanu");
        telemetry.addData("Gamepad1 B", "Reset frente");

        telemetry.addData("vel intake", "%.2f", motorIntake.getPower());
        if (shooterAtivo) {
            telemetry.addData("vel shooter", "%.3f", motorShooter.getPower());
            telemetry.addData("vel shooter2", "%.3f", motorShooter2.getPower());
        }
        telemetry.addData("vel indexer", "%.2f", servoIndexer.getPower());

        telemetry.update();
    }

    private void stopAll() {
        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);
        motorIntake.setPower(0);
        motorShooter.setPower(0);
        motorShooter2.setPower(0);
        servoIndexer.setPower(0);
    }
}