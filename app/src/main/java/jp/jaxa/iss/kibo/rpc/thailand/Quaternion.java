package jp.jaxa.iss.kibo.rpc.thailand;

public class Quaternion {
    private float w, x, y, z;

    // Constructor
    public Quaternion(float w, float x, float y, float z) {
        this.w = w;
        this.x = x;
        this.y = y;
        this.z = z;
    }

    // Getters
    public float getW() { return w; }
    public float getX() { return x; }
    public float getY() { return y; }
    public float getZ() { return z; }

    // Method to multiply two quaternions
    public Quaternion multiply(Quaternion q) {
        float newW = this.w * q.w - this.x * q.x - this.y * q.y - this.z * q.z;
        float newX = this.w * q.x + this.x * q.w + this.y * q.z - this.z * q.y;
        float newY = this.w * q.y - this.x * q.z + this.y * q.w + this.z * q.x;
        float newZ = this.w * q.z + this.x * q.y - this.y * q.x + this.z * q.w;

        return new Quaternion(newW, newX, newY, newZ);
    }

    @Override
    public String toString() {
        return "Quaternion{" +
                "w=" + w +
                ", x=" + x +
                ", y=" + y +
                ", z=" + z +
                '}';
    }
}
