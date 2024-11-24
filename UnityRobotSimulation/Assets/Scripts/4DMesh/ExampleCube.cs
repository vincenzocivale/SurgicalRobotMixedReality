using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ExampleCube : MonoBehaviour
{
    // Start is called before the first frame update
    void Start()
    {
        Mesh4D tesseract = ScriptableObject.CreateInstance<Mesh4D>();
        CreateCube4(tesseract, 0, 1, 0, 1, 0, 1, 0, 1); // Crea un tesseract unitario
    }

    public void CreateCube4(Mesh4D mesh,
    float x0, float x1,
    float y0, float y1,
    float z0, float z1,
    float w0, float w1)
    {
        Vector4[] vertices = new Vector4[8 * 2];
        Mesh4D.Edge[] edges = new Mesh4D.Edge[12 * 2 + 8];

        // w = 0
        // Face down 
        vertices[0] = new Vector4(x0, y0, z0, w0);
        vertices[1] = new Vector4(x1, y0, z0, w0);
        edges[0] = new Mesh4D.Edge(0, 1);

        vertices[2] = new Vector4(x0, y0, z1, w0);
        vertices[3] = new Vector4(x1, y0, z1, w0);
        edges[1] = new Mesh4D.Edge(2, 3);

        edges[2] = new Mesh4D.Edge(0, 2);
        edges[3] = new Mesh4D.Edge(1, 3);

        // Face up
        vertices[4] = new Vector4(x0, y1, z0, w0);
        vertices[5] = new Vector4(x1, y1, z0, w0);
        edges[4] = new Mesh4D.Edge(4, 5);

        vertices[6] = new Vector4(x0, y1, z1, w0);
        vertices[7] = new Vector4(x1, y1, z1, w0);
        edges[5] = new Mesh4D.Edge(6, 7);

        edges[6] = new Mesh4D.Edge(4, 6);
        edges[7] = new Mesh4D.Edge(5, 7);

        // Connects the two faces
        edges[8] = new Mesh4D.Edge(0, 4);
        edges[9] = new Mesh4D.Edge(1, 5);
        edges[10] = new Mesh4D.Edge(2, 6);
        edges[11] = new Mesh4D.Edge(3, 7);

        // w = 0
        // Face down
        vertices[0 + 8] = new Vector4(x0, y0, z0, w1);
        vertices[1 + 8] = new Vector4(x1, y0, z0, w1);
        edges[0 + 12] = new Mesh4D.Edge(0 + 8, 1 + 8);

        vertices[2 + 8] = new Vector4(x0, y0, z1, w1);
        vertices[3 + 8] = new Vector4(x1, y0, z1, w1);
        edges[1 + 12] = new Mesh4D.Edge(2 + 8, 3 + 8);

        edges[2 + 12] = new Mesh4D.Edge(0 + 8, 2 + 8);
        edges[3 + 12] = new Mesh4D.Edge(1 + 8, 3 + 8);

        // Face up
        vertices[4 + 8] = new Vector4(x0, y1, z0, w1);
        vertices[5 + 8] = new Vector4(x1, y1, z0, w1);
        edges[4 + 12] = new Mesh4D.Edge(4 + 8, 5 + 8);

        vertices[6 + 8] = new Vector4(x0, y1, z1, w1);
        vertices[7 + 8] = new Vector4(x1, y1, z1, w1);
        edges[5 + 12] = new Mesh4D.Edge(6 + 8, 7 + 8);

        edges[6 + 12] = new Mesh4D.Edge(4 + 8, 6 + 8);
        edges[7 + 12] = new Mesh4D.Edge(5 + 8, 7 + 8);

        // Connects the two faces
        edges[8 + 12] = new Mesh4D.Edge(0 + 8, 4 + 8);
        edges[9 + 12] = new Mesh4D.Edge(1 + 8, 5 + 8);
        edges[10 + 12] = new Mesh4D.Edge(2 + 8, 6 + 8);
        edges[11 + 12] = new Mesh4D.Edge(3 + 8, 7 + 8);

        // Connects the two cubes
        edges[24] = new Mesh4D.Edge(0, 0 + 8);
        edges[25] = new Mesh4D.Edge(1, 1 + 8);
        edges[26] = new Mesh4D.Edge(2, 2 + 8);
        edges[27] = new Mesh4D.Edge(3, 3 + 8);
        edges[28] = new Mesh4D.Edge(4, 4 + 8);
        edges[29] = new Mesh4D.Edge(5, 5 + 8);
        edges[30] = new Mesh4D.Edge(6, 6 + 8);
        edges[31] = new Mesh4D.Edge(7, 7 + 8);

        // Copies the new geometry
        mesh.Vertices = vertices;
        mesh.Edges = edges;
    }
}
