
package org.firstinspires.ftc.teamcode.pedroPathing

import com.bylazar.configurables.annotations.Configurable
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import dev.nextftc.core.commands.groups.SequentialGroup
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.extensions.pedro.PedroComponent.Companion.follower
import dev.nextftc.ftc.Gamepads
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import dev.nextftc.hardware.impl.MotorEx
import org.firstinspires.ftc.teamcode.next.subsystems.DriveTrain
import org.firstinspires.ftc.teamcode.next.subsystems.DriveTrain.alliance
import org.firstinspires.ftc.teamcode.next.subsystems.Intake
import org.firstinspires.ftc.teamcode.next.subsystems.data.Alliance
import org.firstinspires.ftc.teamcode.next.subsystems.outtake.Turret


@Autonomous
@Configurable
class FarAuto: NextFTCOpMode() {
    init {
        addComponents(
            SubsystemComponent(Intake, DriveTrain, Turret),
            PedroComponent(Constants::createFollower),
            BulkReadComponent,
            BindingsComponent,
        )
    }

    lateinit var poses: Far12

    public override fun onWaitForStart() {
        Gamepads.gamepad1.a whenBecomesTrue { alliance = Alliance.RED }
        Gamepads.gamepad1.b whenBecomesTrue  { alliance = Alliance.BLUE }
        telemetry.run {
            addData("Alliance ", alliance)
            update()
        }
    }

    var frontLeft = MotorEx("motor_esquerda")
    var fR = MotorEx("motor_direita")
    var bR = MotorEx("motor_direitatras")
    var bL = MotorEx("motor_esquerdatras")

    public override fun onInit() {
        follower.setStartingPose(Far12.start)

    }

    override fun onStop() {

        var f: Runnable = Runnable(){
        }
    }

    public override fun onStartButtonPressed() {
        val p: Far12 = Far12(alliance)
        SequentialGroup(
            p.ShootStart,
            /*
            InstantCommand{ Outtake.velocityTrue=true;Outtake.targetVelo = 1350.0; Outtake.hP = 0.0; Outtake.hS.position = 0.0},
            Delay(0.5.seconds),
            Intake.runIntake,
            Delay(0.2.seconds),
            Intake.stopIntake,
            Delay(0.5.seconds),
            Intake.runIntake,
            Delay(0.2.seconds),
            InstantCommand {
                frontLeft.power = 0.5
                fR.power = 0.5
                bL.power = 0.5
                bR.power = .5
            },
            Delay(0.1.seconds),
            InstantCommand {
                frontLeft.power = 0.0
                fR.power = 0.0
                bL.power = 0.0
                bR.power = 0.0
            },
            Delay(0.5.seconds),
            Intake.stopIntake,
            Outtake.flywheelOff,
            InstantCommand {
                frontLeft.power = 0.5
                fR.power = 0.5
                bL.power = 0.5
                bR.power = .5
            },
            Delay(0.5.seconds),
            InstantCommand {
                frontLeft.power = 0.0
                fR.power = 0.0
                bL.power = 0.0
                bR.power = 0.0
            }*/
            /*p.StartToRow1,
           p.Row1Intake,
           p.Row1ToShoot,
           p.ShootToRow2,
           p.Row2Intake,
           p.Row2ToShoot,
           p.ShootToRow3,
           p.Row3Intake,
           p.Row3ToShoot,
           p.ToPark,*/
        ).schedule()
    }


    public override fun onUpdate() {
        telemetry.run {

            addData("follower X", follower.pose.x)
            addData("follower Y", follower.pose.y)
            addData("follower heading", follower.heading)

            update()
        }
    }
}
