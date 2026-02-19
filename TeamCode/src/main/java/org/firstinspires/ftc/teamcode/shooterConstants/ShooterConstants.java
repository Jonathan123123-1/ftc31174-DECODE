package org.firstinspires.ftc.teamcode.shooterConstants;

/**
 * ShooterConstants
 *
 * Provides dynamically calculated control values for the shooter and hood
 * based on a table of calibrated distance-to-RPM/hood-position points.
 * Values are linearly interpolated between the calibrated data points.
 */
public final class ShooterConstants {

    // Master table of calibration points.
    // Each entry is: {distance (in), rpm, hoodPosition}
    private static final double[][] SHOOTER_DATA = {
            // --- Big Close Triangle ---
            {55, 1155, 0.157},
            {56, 1155, 0.157},
            {57, 1155, 0.157},
            {58, 1155, 0.157},
            {59, 1200, 0.157},
            {60, 1200, 0.157},
            {61, 1220, 0.157},
            {62, 1250, 0.157},
            {63, 1250, 0.157},
            {64, 1250, 0.157},
            {65, 1260, 0.157},
            {66, 1260, 0.157},
            {67, 1260, 0.157},
            {68, 1260, 0.157},
            {69, 1260, 0.157},
            {70, 1270, 0.157},
            {71, 1270, 0.157},
            {72, 1270, 0.157},
            {73, 1270, 0.157},
            {74, 1270, 0.157},
            {75, 1270, 0.157},
            {76, 1270, 0.157},
            {77, 1270, 0.157},
            {78, 1280, 0.157},
            {79, 1280, 0.157},
            {80, 1280, 0.157},
            {81, 1310, 0.157},
            {82, 1310, 0.157},
            {83, 1310, 0.157},
            {84, 1310, 0.157},
            {85, 1310, 0.157},
            {86, 1310, 0.157},
            {87, 1350, 0.157},
            {88, 1350, 0.157},
            {89, 1350, 0.157},
            {90, 1350, 0.157},
            {91, 1350, 0.157},
            {92, 1360, 0.157},

            // --- Middle Points (Cant Shoot) for better accuracy ---
            {93, 1360, 0.157},
            {94, 1360, 0.157},
            {95, 1360, 0.157},
            {96, 1360, 0.157},
            {97, 1360, 0.157},
            {98, 1360, 0.157},
            {99, 1360, 0.157},
            {100, 1400, 0.211},
            {101, 1400, 0.211},
            {102, 1400, 0.211},
            {103, 1400, 0.211},
            {104, 1400, 0.211},
            {105, 1410, 0.211},
            {106, 1410, 0.211},
            {107, 1430, 0.211},
            {108, 1430, 0.211},
            {109, 1430, 0.211},
            {110, 1430, 0.211},
            {111, 1430, 0.211},
            {112, 1435, 0.211},
            {113, 1435, 0.211},
            {114, 1435, 0.211},
            {115, 1435, 0.211},
            {116, 1400, 0.211},
            {117, 1400, 0.211},
            {118, 1400, 0.211},

            // --- Back Small Triangle ---
            {119, 1450, 0.211},
            {120, 1455, 0.211},
            {121, 1455, 0.211},
            {122, 1455, 0.211},
            {123, 1455, 0.211},
            {124, 1455, 0.211},
            {125, 1460, 0.211},
            {126, 1460, 0.211},
            {127, 1460, 0.211},
            {128, 1465, 0.211},
            {129, 1465, 0.211},
            {130, 1465, 0.211},
            {131, 1465, 0.211},
            {132, 1465, 0.211},
            {133, 1465, 0.211},
            {134, 1465, 0.211},
            {135, 1475, 0.211},
            {136, 1475, 0.211},
            {137, 1480, 0.211},
            {138, 1480, 0.211},
            {139, 1480, 0.211},
            {140, 1490, 0.211},
            {141, 1490, 0.211},
            {142, 1490, 0.211},
            {143, 1490, 0.211},
            {144, 1490, 0.211},
            {145, 1500, 0.211},
            {146, 1500, 0.211},
            {147, 1500, 0.211},
            {148, 1500, 0.211},
            {149, 1500, 0.211},
            {150, 1520, 0.211},
            {151, 1520, 0.211},
            {152, 1520, 0.211},
            {153, 1520, 0.211},
            {154, 1520, 0.211},
            {155, 1520, 0.211},
            {156, 1535, 0.211},
            {157, 1535, 0.211},
            {158, 1535, 0.211},
            {159, 1535, 0.211},
            {160, 1535, 0.211},
            {161, 1535, 0.211},
            {162, 1535, 0.211}
    };

    private ShooterConstants() {}

    /**
     * Calculates the target hood position based on the distance to the target.
     * It finds the nearest entry in the SHOOTER_DATA table to determine the hood position.
     * This is designed to work with step-like changes in hood position from the table.
     * @param distance The distance to the target in inches.
     * @return The calculated hood servo position.
     */
    public static double hoodPosition(double distance) {
        // Handle distances outside the calibrated range
        if (distance <= SHOOTER_DATA[0][0]) {
            return SHOOTER_DATA[0][2];
        }
        if (distance >= SHOOTER_DATA[SHOOTER_DATA.length - 1][0]) {
            return SHOOTER_DATA[SHOOTER_DATA.length - 1][2];
        }

        // Find the two data points that bracket the current distance
        int i = 0;
        while (i < SHOOTER_DATA.length - 2 && distance > SHOOTER_DATA[i + 1][0]) {
            i++;
        }

        double d1 = SHOOTER_DATA[i][0];
        double h1 = SHOOTER_DATA[i][2];
        double d2 = SHOOTER_DATA[i + 1][0];
        double h2 = SHOOTER_DATA[i + 1][2];

        // If hood positions are the same, no need to check which is closer
        if (h1 == h2) {
            return h1;
        }

        // Return the hood position of the nearest distance entry
        if ((distance - d1) < (d2 - distance)) {
            return h1;
        } else {
            return h2;
        }
    }

    /**
     * Calculates the target shooter RPM based on the distance to the target.
     * It linearly interpolates between the points defined in the SHOOTER_DATA table.
     * For distances outside the table or in the gap, it uses the RPM of the nearest data point.
     * @param distance The distance to the target in inches.
     * @return The calculated target RPM.
     */
    public static double targetRPM(double distance) {
        // Handle distances outside the calibrated range
        if (distance <= SHOOTER_DATA[0][0]) {
            return SHOOTER_DATA[0][1];
        }
        if (distance >= SHOOTER_DATA[SHOOTER_DATA.length - 1][0]) {
            return SHOOTER_DATA[SHOOTER_DATA.length - 1][1];
        }

        // Find the two data points that bracket the current distance
        int i = 0;
        while (i < SHOOTER_DATA.length - 2 && distance > SHOOTER_DATA[i + 1][0]) {
            i++;
        }

        double d1 = SHOOTER_DATA[i][0];
        double r1 = SHOOTER_DATA[i][1];
        double d2 = SHOOTER_DATA[i + 1][0];
        double r2 = SHOOTER_DATA[i + 1][1];

        // Check if the distance falls in the gap between the two triangles (index 7 to 8)
        if (i == 7 && distance > d1 && distance < d2) {
            // Return the RPM of the closer endpoint of the gap
            return (distance - d1 < d2 - distance) ? r1 : r2;
        }

        // Perform linear interpolation
        return interpolate(distance, d1, d2, r1, r2);
    }

    /**
     * Performs linear interpolation between two points.
     */
    private static double interpolate(double x, double x1, double x2, double y1, double y2) {
        if (x1 == x2) return y1;
        return y1 + (x - x1) * (y2 - y1) / (x2 - x1);
    }
}
