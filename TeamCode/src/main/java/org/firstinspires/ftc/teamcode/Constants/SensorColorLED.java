package org.firstinspires.ftc.teamcode.Constants;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
@Configurable
public class SensorColorLED {

    public static float gain = 70f;
    public static float threshold = 0.1f;

    public static double azul = 0.61;
    public static double branco = 0.50;
    public static double vermelho = 0.28;

    public static volatile float a1;
    public static volatile float a2;
    public static volatile float a3;

    public static volatile int count;
    public static float getA1() { return a1; }
    public static float getA2() { return a2; }
    public static float getA3() { return a3; }

    public static volatile boolean running = false;

    public static void startThread(
            final NormalizedColorSensor s1,
            final NormalizedColorSensor s2,
            final NormalizedColorSensor s3,
            final Servo led
    ) {

        running = true;

        Thread t = new Thread(() -> {

            int stable = 0;
            int blink = 0;
            boolean state = false;

            while (running) {

                s1.setGain(gain);
                s2.setGain(gain);
                s3.setGain(gain);

                a1 = s1.getNormalizedColors().alpha;
                a2 = s2.getNormalizedColors().alpha;
                a3 = s3.getNormalizedColors().alpha;

                count =
                        (a1 > threshold ? 1 : 0) +
                                (a2 > threshold ? 1 : 0) +
                                (a3 > threshold ? 1 : 0);

                if (count == 3) {

                    stable++;

                    if (stable > 2) {

                        blink++;

                        if (blink > 1) {
                            state = !state;
                            blink = 0;
                        }

                        led.setPosition(state ? azul : branco);

                    } else {
                        led.setPosition(azul);
                    }

                } else {

                    stable = 0;
                    blink = 0;

                    led.setPosition(vermelho);
                }

                try {
                    Thread.sleep(5);
                } catch (InterruptedException ignored) {}
            }
        });

        t.start();
    }

    public static void stop() {
        running = false;
    }
}