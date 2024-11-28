using System.Collections.Generic;
using System.Linq;
using UnityEngine;

/// <summary>
/// Vertex Pair maps two vertices to each other, used to find the nearest vertex to a given vertex.
/// </summary>
public class VertexMapping
{
    public Vector3 Vertex1 { get; set; }
    public Vector3 Vertex2 { get; set; }

    public VertexMapping(Vector3 vertex1, Vector3 vertex2)
    {
        Vertex1 = vertex1;
        Vertex2 = vertex2;
    }
}

/// <summary>
/// MeshDeformer handles morphing between multiple meshes in sequence with a shared material.
/// </summary>
public class MeshDeformer : MonoBehaviour
{
    public bool IsDeforming = true;

    [Tooltip("Meshes should be read/write enabled from the model import settings")]
    [SerializeField] private List<Mesh> _meshes;

    [Tooltip("Shared material for all meshes")]
    [SerializeField] private Material _sharedMaterial;

    [SerializeField] private MeshFilter _meshFilter;
    [SerializeField] private Renderer _renderer;

    [Tooltip("Transition speed (units per second)")]
    [SerializeField] private float _transitionSpeed = 1.0f;

    private List<Vector3[]> _verticesList;
    private List<int[]> _trianglesList;

    private Mesh _interpolatedMesh;
    private int _currentMeshIndex = 0;
    private float _slider = 0.0f;

    private List<VertexMapping> _vertexPairs;

    void Start()
    {
        if (_meshes.Count < 2)
        {
            Debug.LogError("You must provide at least 2 meshes!");
            return;
        }

        if (_sharedMaterial == null)
        {
            Debug.LogError("A shared material must be assigned!");
            return;
        }

        for (int i = _meshes.Count - 2; i > 0; i--)
        {
            _meshes.Add(_meshes[i]);
        }

        _interpolatedMesh = new Mesh();
        _interpolatedMesh.MarkDynamic();
        _meshFilter.mesh = _interpolatedMesh;

        _verticesList = _meshes.Select(mesh => mesh.vertices).ToList();
        _trianglesList = _meshes.Select(mesh => mesh.triangles).ToList();

        // Assign the shared material to the renderer
        _renderer.material = _sharedMaterial;

        CreateVertexPairs();
    }

    /// <summary>
    /// Creates pairs of vertices for the current mesh transition.
    /// </summary>
    private void CreateVertexPairs()
    {
        _vertexPairs = new List<VertexMapping>();
        var currentVertices = _verticesList[_currentMeshIndex];
        var nextVertices = _verticesList[(_currentMeshIndex + 1) % _verticesList.Count];

        foreach (var vertex in currentVertices)
        {
            var nearestVertex = nextVertices.OrderBy(v => Vector3.Distance(v, vertex)).FirstOrDefault();
            _vertexPairs.Add(new VertexMapping(vertex, nearestVertex));
        }
    }

    void Update()
    {
        if (IsDeforming)
        {
            Deform();
        }
    }

    /// <summary>
    /// Handles the morphing logic.
    /// </summary>
    private void Deform()
    {
        _slider += Time.deltaTime * _transitionSpeed;

        if (_slider >= 1.0f)
        {
            _slider = 0.0f;
            _currentMeshIndex = (_currentMeshIndex + 1) % _meshes.Count;
            CreateVertexPairs();
        }

        var interpolatedVertices = _vertexPairs.Select(p => Vector3.Lerp(p.Vertex1, p.Vertex2, _slider)).ToList();

        _interpolatedMesh.Clear();
        _interpolatedMesh.SetVertices(interpolatedVertices);
        _interpolatedMesh.triangles = _trianglesList[_currentMeshIndex];
        _interpolatedMesh.bounds = _meshes[_currentMeshIndex].bounds;
        _interpolatedMesh.uv = _meshes[_currentMeshIndex].uv;

        _interpolatedMesh.RecalculateNormals();
    }
}