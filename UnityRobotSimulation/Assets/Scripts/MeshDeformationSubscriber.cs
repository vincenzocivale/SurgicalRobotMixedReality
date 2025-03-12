using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.SofaSurgical;
using System.Collections.Generic;

public class MeshDeformationSubscriber : MonoBehaviour
{
    public string topic_name;
    private string nodeName;  // Nome del GameObject da cercare
    private MeshFilter meshFilter;
    private Mesh mesh;
    private Vector3[] originalVertices;

    private double lastTimestamp = 0.0; // Timestamp del messaggio precedente
    private float fps = 0.0f; // Frame rate calcolat

    void Start()
    {
        // Sottoscrizione al topic ROS
        ROSConnection.GetOrCreateInstance().Subscribe<DeformationUpdateMsg>(topic_name, HandleDeformationUpdate);

        // Trova il GameObject nella scena
        GameObject node = GameObject.Find(nodeName);
        if (node != null)
        {
            meshFilter = node.GetComponent<MeshFilter>();
            if (meshFilter != null)
            {
                mesh = meshFilter.mesh;
                originalVertices = mesh.vertices; 
            }
            else
            {
                Debug.LogError("Il GameObject non ha un MeshFilter.");
            }
        }
        else
        {
            Debug.LogError("GameObject con nome " + nodeName + " non trovato nella scena.");
        }

    }

    // Funzione di callback per il topic DeformationUpdate
    void HandleDeformationUpdate(DeformationUpdateMsg deformationUpdate)
    {
            GameObject node = GameObject.Find(deformationUpdate.node_name);
            if (node != null)
            {
                meshFilter = node.GetComponent<MeshFilter>();
                if (meshFilter != null)
                {
                    mesh = meshFilter.mesh;
                    originalVertices = mesh.vertices;
                }
                else
                {
                    Debug.LogError("Il GameObject non ha un MeshFilter.");
                }
            }
            else
            {
                Debug.LogError("GameObject con nome " + nodeName + " non trovato nella scena.");
            }


           Vector3[] vertices = mesh.vertices;

            // Aggiorna i vertici della mesh in base ai displacement
            for (int i = 0; i < deformationUpdate.vertex_ids.Length; i++)
            {
                int vertexId = deformationUpdate.vertex_ids[i];
                if (vertexId >= 0 && vertexId < vertices.Length)
                {
                    // Applica il displacement
                    DisplacementMsg displacement = deformationUpdate.displacements[i];
                    Vector3 displacementVector = new Vector3((float)displacement.dx, (float)displacement.dy, -(float)displacement.dz);
                    vertices[vertexId] += displacementVector;  // Modifica la posizione del vertice
                }
            }

            // Applica i nuovi vertici alla mesh
            mesh.vertices = vertices;
            mesh.RecalculateNormals();
            mesh.RecalculateBounds();


            if (lastTimestamp > 0)
            {
                double deltaTime = deformationUpdate.timestamp - lastTimestamp;
                if (deltaTime > 0)
                {
                    fps = (float)(1.0 / deltaTime);
                }
                else
                {
                    Debug.LogError("Messaggio uguale");
                }
        }
            lastTimestamp = deformationUpdate.timestamp;

    }

    

    void OnGUI()
    {
        // Mostra il frame rate nell'editor di Unity
        GUIStyle style = new GUIStyle();
        style.fontSize = 20;
        style.normal.textColor = Color.white;
        GUI.Label(new Rect(10, 10, 300, 30), "ROS Mesh Update FPS: " + fps.ToString("F2"), style);
    }


}
