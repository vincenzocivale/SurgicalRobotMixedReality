using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public static class Matrix4x4Extensions
{
    public static Vector4 Multiply(this Matrix4x4 matrix, Vector4 vector)
    {
        return new Vector4(
            matrix.m00 * vector.x + matrix.m01 * vector.y + matrix.m02 * vector.z + matrix.m03 * vector.w,
            matrix.m10 * vector.x + matrix.m11 * vector.y + matrix.m12 * vector.z + matrix.m13 * vector.w,
            matrix.m20 * vector.x + matrix.m21 * vector.y + matrix.m22 * vector.z + matrix.m23 * vector.w,
            matrix.m30 * vector.x + matrix.m31 * vector.y + matrix.m32 * vector.z + matrix.m33 * vector.w
        );
    }

    public static Matrix4x4 RotateXY(this Matrix4x4 m, float a)
    {
        return m * RotateXY(a);
    }

    public static Matrix4x4 RotateYZ(this Matrix4x4 m, float a)
    {
        return m * RotateYZ(a);
    }

    public static Matrix4x4 RotateXZ(this Matrix4x4 m, float a)
    {
        return m * RotateXZ(a);
    }

    public static Matrix4x4 RotateXW(this Matrix4x4 m, float a)
    {
        return m * RotateXW(a);
    }

    public static Matrix4x4 RotateYW(this Matrix4x4 m, float a)
    {
        return m * RotateYW(a);
    }

    public static Matrix4x4 RotateZW(this Matrix4x4 m, float a)
    {
        return m * RotateZW(a);
    }

    public static Matrix4x4 RotateXY(float a)
    {
        float c = Mathf.Cos(a);
        float s = Mathf.Sin(a);
        Matrix4x4 m = new Matrix4x4();
        m.SetColumn(0, new Vector4(c, -s, 0, 0));
        m.SetColumn(1, new Vector4(s, c, 0, 0));
        m.SetColumn(2, new Vector4(0, 0, 1, 0));
        m.SetColumn(3, new Vector4(0, 0, 0, 1));
        return m;
    }

    public static Matrix4x4 RotateYZ(float a)
    {
        float c = Mathf.Cos(a);
        float s = Mathf.Sin(a);
        Matrix4x4 m = new Matrix4x4();
        m.SetColumn(0, new Vector4(1, 0, 0, 0));
        m.SetColumn(1, new Vector4(0, c, -s, 0));
        m.SetColumn(2, new Vector4(0, s, c, 0));
        m.SetColumn(3, new Vector4(0, 0, 0, 1));
        return m;
    }

    public static Matrix4x4 RotateXZ(float a)
    {
        float c = Mathf.Cos(a);
        float s = Mathf.Sin(a);
        Matrix4x4 m = new Matrix4x4();
        m.SetColumn(0, new Vector4(c, 0, s, 0));
        m.SetColumn(1, new Vector4(0, 1, 0, 0));
        m.SetColumn(2, new Vector4(-s, 0, c, 0));
        m.SetColumn(3, new Vector4(0, 0, 0, 1));
        return m;
    }

    public static Matrix4x4 RotateXW(float a)
    {
        float c = Mathf.Cos(a);
        float s = Mathf.Sin(a);
        Matrix4x4 m = new Matrix4x4();
        m.SetColumn(0, new Vector4(c, 0, 0, -s));
        m.SetColumn(1, new Vector4(0, 1, 0, 0));
        m.SetColumn(2, new Vector4(0, 0, 1, 0));
        m.SetColumn(3, new Vector4(s, 0, 0, c));
        return m;
    }

    public static Matrix4x4 RotateYW(float a)
    {
        float c = Mathf.Cos(a);
        float s = Mathf.Sin(a);
        Matrix4x4 m = new Matrix4x4();
        m.SetColumn(0, new Vector4(1, 0, 0, 0));
        m.SetColumn(1, new Vector4(0, c, 0, s));
        m.SetColumn(2, new Vector4(0, 0, 1, 0));
        m.SetColumn(3, new Vector4(0, -s, 0, c));
        return m;
    }

    public static Matrix4x4 RotateZW(float a)
    {
        float c = Mathf.Cos(a);
        float s = Mathf.Sin(a);
        Matrix4x4 m = new Matrix4x4();
        m.SetColumn(0, new Vector4(1, 0, 0, 0));
        m.SetColumn(1, new Vector4(0, 1, 0, 0));
        m.SetColumn(2, new Vector4(0, 0, c, s));
        m.SetColumn(3, new Vector4(0, 0, -s, c));
        return m;
    }
}

