using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UIElements;


public class Transform4D : MonoBehaviour
{
    [Header("Mesh4D")]
    public Mesh4D Mesh;
    private Vector4[] Vertices;
    [Header("Transform")]
    public Vector4 Position;
    public Euler4 Rotation;
    public Vector4 Scale = new Vector4(1, 1, 1, 1);
    private Matrix4x4 RotationMatrix;
    private Matrix4x4 RotationInverse;

    private void Update()
    {
        UpdateRotationMatrix();
        UpdateVertices();
    }

    private void UpdateVertices()
    {
        for (int i = 0; i < Mesh.Vertices.Length; i++)
            Vertices[i] = Transform(Mesh.Vertices[i]);
    }

    // Takes a 4D point and translate, rotate and scale it
    // according to this transform
    public Vector4 Transform(Vector4 v)
    {
        // Rotates around zero
        v = RotationMatrix.Multiply(v);

        // Scales around zero
        v.x *= Scale.x;
        v.y *= Scale.y;
        v.z *= Scale.z;
        v.w *= Scale.w;

        // Translates
        v += Position;

        return v;
    }

    private Matrix4x4 UpdateRotationMatrix()
    {
        RotationMatrix =
            Matrix4x4.identity
            .RotateXY(Rotation.XY * Mathf.Deg2Rad)
            .RotateYZ(Rotation.YZ * Mathf.Deg2Rad)
            .RotateXZ(Rotation.XZ * Mathf.Deg2Rad)
            .RotateXW(Rotation.XW * Mathf.Deg2Rad)
            .RotateYW(Rotation.YW * Mathf.Deg2Rad)
            .RotateZW(Rotation.ZW * Mathf.Deg2Rad);
        RotationInverse = RotationMatrix.inverse;

        return RotationMatrix;
    }
    
}

[Serializable]
public struct Euler4
{
    [Range(-180, +180)]
    public float XY; // Z (W)
    [Range(-180, +180)]
    public float YZ; // X (w)
    [Range(-180, +180)]
    public float XZ; // Y (W)
    [Range(-180, +180)]
    public float XW; // Y Z
    [Range(-180, +180)]
    public float YW; // X Z
    [Range(-180, +180)]
    public float ZW; // X Y
}
