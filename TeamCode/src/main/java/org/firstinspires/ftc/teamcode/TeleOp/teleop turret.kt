import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.ftc.Gamepads
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import dev.nextftc.hardware.driving.MecanumDriverControlled
import dev.nextftc.hardware.impl.MotorEx
import org.firstinspires.ftc.teamcode.next.subsystems.DriveTrain
import org.firstinspires.ftc.teamcode.next.subsystems.outtake.Turret

@TeleOp(name = "Fixed Turret TeleOp")
class teleopturret : NextFTCOpMode() {

    init {
        addComponents(
            SubsystemComponent(DriveTrain, Turret),
            BulkReadComponent,
            BindingsComponent
        )
    }

    private val frontLeftMotor = MotorEx("motor_esquerda").reversed()
    private val frontRightMotor = MotorEx("motor_direita")
    private val backLeftMotor = MotorEx("motor_esquerdatras").reversed()
    private val backRightMotor = MotorEx("motor_direitatras")

    override fun onStartButtonPressed() {
        val driverControlled = MecanumDriverControlled(
            frontLeftMotor,
            frontRightMotor,
            backLeftMotor,
            backRightMotor,
            -Gamepads.gamepad1.leftStickY.map { it * DriveTrain.sensitivity },
            Gamepads.gamepad1.leftStickX.map { it * DriveTrain.sensitivity },
            Gamepads.gamepad1.rightStickX.map { it * DriveTrain.sensitivity }
        )
        driverControlled()

        Gamepads.gamepad1.a.whenBecomesTrue {
            Turret.autoTurret = false
            Turret.goToYaw(0.0)
        }

        Gamepads.gamepad1.b.whenBecomesTrue {
            Turret.autoTurret = true
        }
    }
}