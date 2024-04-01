using UnityEngine;

public static class Utilities
{
    public static Vector3 AsNormalizedEulerAngles(Quaternion rotation)
    {
        return rotation.eulerAngles / 180.0f - Vector3.one;
    }

    public static float MapToInterval(float val, float valmin, float valmax, float min, float max)
    {
        return ((val - valmin) / (valmax - valmin)) * (max - min) + min;
    }

    public static Vector3 GetRandomPosition(float centerX, float centerZ, float radiusMin, float radiusMax, float radiansMin, float radiansMax)
    {
        float radius = Mathf.Sqrt(Random.Range(Mathf.Pow(radiusMin, 2), Mathf.Pow(radiusMax, 2)));  // random radius
        float angle = Random.Range(radiansMin, radiansMax) * Mathf.PI;  // random angle
        float x = radius * Mathf.Cos(angle);  // to cartesian
        float z = radius * Mathf.Sin(angle); // to cartesian
        return new Vector3(centerX + x, 6.41f, centerZ + z);
    }

    public static bool IsWithinPartialCircle(float centerX, float centerZ, float radiusMin, float radiusMax, float radiansMin, float radiansMax, float toTestX, float toTestZ)
    {
        float radiusToTest = Mathf.Sqrt(Mathf.Pow(toTestX-centerX, 2) + Mathf.Pow(toTestZ-centerZ, 2)); // to polar
        float angleToTest = Mathf.Atan2(toTestZ, toTestX) / Mathf.PI; // to polar
        return angleToTest > radiansMin 
            && angleToTest < radiansMax 
            && radiusToTest > radiusMin 
            && radiusToTest < radiusMax;
    }

    public static float GetCircularDistance(Transform center, Transform other)
    {
        return Mathf.Sqrt(Mathf.Pow((other.position.x - center.position.x), 2) + Mathf.Pow((other.position.z - center.position.z), 2));
    }

    public static Quaternion GetRotationDifference(Quaternion from, Quaternion to)
    {
        // https://forum.unity.com/threads/get-the-difference-between-two-quaternions-and-add-it-to-another-quaternion.513187/
        return (to * Quaternion.Inverse(from)) * from;
    }
}
