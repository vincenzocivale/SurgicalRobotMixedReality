using System;
using System.Collections.Generic;
using UnityEngine;
using MIConvexHull;

public class Mesh4D : ScriptableObject
{
    public Vector4[] Vertices;
    public Edge[] Edges;

    [Serializable]
    public struct Edge
    {
        public int Index0;
        public int Index1;

        public Edge(int index0, int index1)
        {
            Index0 = index0;
            Index1 = index1;
        }
    }

    // finds the 3D intersection of a 4D segment with endpoints v0 and v1
    private int Intersection(List<Vector4> list, Vector4 v0, Vector4 v1)
    {
        // Both points are 3D ==> the entire segment lies in the 3D space
        if (v1.w == 0 && v0.w == 0)
        {
            list.Add(v0);
            list.Add(v1);
            return 2;
        }

        // Both w coordinates are equale
        // If they are both 0 ==> the entire line is in the 3D space (already tested)
        // If they are not 0 ==> the entire line is outside the 3D space
        if (v1.w - v0.w == 0)
            return 0;

        // Time of intersection
        float t = -v0.w / (v1.w - v0.w);

        // No intersection
        if (t < 0 || t > 1)
            return 0;

        // One intersection
        Vector4 x = v0 + (v1 - v0) * t;
        list.Add(x);
        return 1;
    }

    public Mesh Intersect()
    {
        // Calculates the intersections
        List<Vector4> vertices = new List<Vector4>();
        foreach (Edge edge in this.Edges)
            Intersection
            (
                vertices,
                this.Vertices[edge.Index0],
                this.Vertices[edge.Index1]
            );

        // Not enough intersection points!
        if (vertices.Count < 3)
            return null;

        // Creates and returns the mesh
        return CreateMesh(vertices);
    }

    public class Vertex3 : IVertex
    {
        public double[] Position { get; private set; }

        public Vertex3(Vector3 position)
        {
            Position = new double[3] { position.x, position.y, position.z };
        }
    }

    Mesh CreateMesh(List<Vector4> vertex4)
    {
        // Convert Vector4 to Vector3
        List<Vertex3> vertices = new List<Vertex3>();
        foreach (var v in vertex4)
        {
            vertices.Add(new Vertex3(new Vector3(v.x, v.y, v.z)));
        }

        // Creates the convex hull
        var hull = ConvexHull.Create(vertices);
        var faces = hull.Result.Faces;

        // Extract mesh data
        List<Vector3> vertices3 = new List<Vector3>();
        List<int> triangles = new List<int>();

        foreach (var face in faces)
        {
            var v0 = new Vector3((float)face.Vertices[0].Position[0], (float)face.Vertices[0].Position[1], (float)face.Vertices[0].Position[2]);
            var v1 = new Vector3((float)face.Vertices[1].Position[0], (float)face.Vertices[1].Position[1], (float)face.Vertices[1].Position[2]);
            var v2 = new Vector3((float)face.Vertices[2].Position[0], (float)face.Vertices[2].Position[1], (float)face.Vertices[2].Position[2]);

            int idx0 = vertices3.Count;
            vertices3.Add(v0);
            vertices3.Add(v1);
            vertices3.Add(v2);

            triangles.Add(idx0);
            triangles.Add(idx0 + 1);
            triangles.Add(idx0 + 2);
        }

        // Create Unity mesh
        Mesh mesh = new Mesh();
        mesh.vertices = vertices3.ToArray();
        mesh.triangles = triangles.ToArray();
        mesh.RecalculateNormals();

        return mesh;
    }


}
