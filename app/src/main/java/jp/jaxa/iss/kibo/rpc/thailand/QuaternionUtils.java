package jp.jaxa.iss.kibo.rpc.thailand;

import java.util.List;

public class QuaternionUtils {

    public static Quaternion eulerToQuaternion_use(float roll, float pitch, float yaw) {
        // Convert degrees to radians
        float rollRad = (float) Math.toRadians(roll);
        float pitchRad = (float) Math.toRadians(pitch);
        float yawRad = (float) Math.toRadians(yaw);

        // Calculate the quaternion components
        float cy = (float) Math.cos(yawRad * 0.5);
        float sy = (float) Math.sin(yawRad * 0.5);
        float cp = (float) Math.cos(pitchRad * 0.5);
        float sp = (float) Math.sin(pitchRad * 0.5);
        float cr = (float) Math.cos(rollRad * 0.5);
        float sr = (float) Math.sin(rollRad * 0.5);

        float w = cy * cp * cr + sy * sp * sr;
        float x = cy * cp * sr - sy * sp * cr;
        float y = sy * cp * sr + cy * sp * cr;
        float z = sy * cp * cr - cy * sp * sr;

        return new Quaternion(w, x, y, z);
    }

    public static Quaternion computeQuaternionFromAngles(List<Double> degrees) {
        // Ensure there are exactly three angles
        if (degrees.size() != 3) {
            throw new IllegalArgumentException("List must contain exactly three angles.");
        }

        float z = degrees.get(0).floatValue();
        float y = degrees.get(1).floatValue();
        float x = degrees.get(2).floatValue();

        // Create quaternions for each rotation
        Quaternion qZ = eulerToQuaternion_use(0.0f, 0.0f, z);
        Quaternion qY = eulerToQuaternion_use(0.0f, y, 0.0f);
        Quaternion qX = eulerToQuaternion_use(x, 0.0f, 0.0f);

        // Multiply the quaternions together
        Quaternion result = qY.multiply(qZ);
        result = result.multiply(qX);

        return result;
    }
}