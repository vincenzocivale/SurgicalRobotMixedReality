using RosMessageTypes.SofaSurgical;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using System.Linq;
using RosMessageTypes.Shape;

public class RosServiceCallExample : MonoBehaviour
{
    private ROSConnection ros;

    // Nome del servizio che stai chiamando
    public string serviceName = "/get_organ";

    void Start()
    {
        // Crea e connetti la connessione ROS
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterRosService<GetOrganRequest, GetOrganResponse>(serviceName);

        // Chiamata al servizio all'inizio
        CallGetOrganService();
    }

    private void CallGetOrganService()
    {
        // Crea una richiesta per il servizio
        GetOrganRequest request = new GetOrganRequest();

        // Esegui la chiamata al servizio e ricevi la risposta
        ros.SendServiceMessage<GetOrganResponse>(serviceName, request, Callback_OrganResponse);
    }

    private void Callback_OrganResponse(GetOrganResponse response)
    {
        OrganMsg organData = response.organ; // Assegna i dati restituiti nel messaggio Organ

        // Log dei dati ricevuti
        Debug.Log($"Received Organ Data: {organData.id}");
        Debug.Log($"Organ Position: {organData.pose.position.x}, {organData.pose.position.y}, {organData.pose.position.z}");

        // Creazione dell'oggetto Organ
        GameObject organObject = new GameObject(organData.id);
        organObject.transform.position = new Vector3(
            (float)organData.pose.position.x,
            (float)organData.pose.position.y,
            (float)organData.pose.position.z
        );

        // Aggiunge il componente MeshFilter e MeshRenderer
        organObject.AddComponent<MeshFilter>();
        organObject.AddComponent<MeshRenderer>();

        // Genera la mesh
        GenerateMesh(organObject, organData.surface);
    }

    private void GenerateMesh(GameObject organObject, MeshMsg surface)
    {
        Mesh mesh = new Mesh();

        // Converti i vertici da ROS a Unity
        Vector3[] vertices = surface.vertices
            .Select(v => new Vector3((float)v.x, (float)v.y, (float)v.z))
            .ToArray();
        mesh.vertices = vertices;

        // Converti gli indici delle facce
        int[] triangles = surface.triangles
            .SelectMany(t => t.vertex_indices.Select(i => (int)i))
            .ToArray();
        mesh.triangles = triangles;

        // Ricalcola normali per un rendering corretto
        mesh.RecalculateNormals();

        // Applica la mesh all'oggetto
        MeshFilter meshFilter = organObject.GetComponent<MeshFilter>();
        meshFilter.mesh = mesh;

        // Aggiunge un materiale di base
        MeshRenderer meshRenderer = organObject.GetComponent<MeshRenderer>();
        meshRenderer.material = new Material(Shader.Find("Standard"));
    }
}
