using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.EcmController;
using System.Linq;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using RosMessageTypes.Geometry;
using RosMessageTypes.Shape;

public class PatientMeshController : MonoBehaviour
{

    [SerializeField]
    private string patientMeshTopic;

    // ROS Connector
    ROSConnection ros;
    float lastSent;

    [SerializeField]
    private float rate;

    [SerializeField]
    private GameObject robot;

    [SerializeField]
    private GameObject hologramObstacles;

    MeshFilter[] hologramObstacleMeshFilters;

    // Start is called before the first frame update
    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<PatientMeshMsg>(patientMeshTopic);

        lastSent = Time.time;

    }

    // Update is called once per frame
    void Update()
    {

        if (Time.time - lastSent > 1 / rate)
        {
            PublishHologramMeshes();
        }

    }

    void PublishHologramMeshes()
    {
        hologramObstacleMeshFilters = hologramObstacles.GetComponentsInChildren<MeshFilter>();
        PatientMeshMsg[] hologramObstacleMsgs = new PatientMeshMsg[hologramObstacleMeshFilters.Length];

        int obstacleIdx = 0;
        foreach (var hologramFilter in hologramObstacleMeshFilters)
        {
            PatientMeshMsg meshObstacleMsg = new PatientMeshMsg
            {
                reconstructed_mesh = MeshToMeshMsg(hologramFilter.mesh, hologramFilter.gameObject),
                mesh_pose = PoseInRobotFrameMsg(robot, hologramFilter.gameObject),
                patient_id = hologramFilter.gameObject.name
            };

            hologramObstacleMsgs[obstacleIdx++] = meshObstacleMsg;

        }

        ros.Publish(patientMeshTopic, new PatientMeshArrayMsg(hologramObstacleMsgs));
    }

    public void AddHologramObstacle(GameObject obstaclePrefab)
    {
        GameObject hologramObstacle = Instantiate(obstaclePrefab);
        hologramObstacle.name = System.Guid.NewGuid().ToString();
        hologramObstacle.transform.SetParent(hologramObstacles.transform);

        Vector3 inFrontOfCameraPos = (Camera.main.transform.forward * 0.5f) + Camera.main.transform.position;
        hologramObstacle.transform.position = inFrontOfCameraPos;
    }

    public void ClearHologramObstacles()
    {
        foreach (Transform obstacleTransform in hologramObstacles.transform)
        {
            Destroy(obstacleTransform.gameObject);
        }
    }

    public static MeshMsg MeshToMeshMsg(Mesh meshObject, GameObject gameObject)
    {
        Vector3[] vertices = meshObject.vertices;
        uint[] triangles = meshObject.triangles.Select(triangle => (uint)triangle).ToArray();

        PointMsg[] vertexPoints = new PointMsg[vertices.Length];

        for (int vertexIdx = 0; vertexIdx < vertexPoints.Length; vertexIdx++)
        {
            Vector3 vertexPoint = new Vector3(vertices[vertexIdx].x * gameObject.transform.localScale.x, vertices[vertexIdx].y * gameObject.transform.localScale.y, vertices[vertexIdx].z * gameObject.transform.localScale.z);
            Vector3<FLU> fluVertexPoint = vertexPoint.To<FLU>();
            vertexPoints[vertexIdx] = new PointMsg(fluVertexPoint.x, fluVertexPoint.y, fluVertexPoint.z);
        }

        MeshTriangleMsg[] meshTriangleMsgs = new MeshTriangleMsg[triangles.Length / 3];

        int trianglePointIter = 0;
        for (int meshTriangleIdx = 0; meshTriangleIdx < meshTriangleMsgs.Length; meshTriangleIdx++)
        {
            // flipping order of trinagle points so that normals are correct in robot frame
            uint[] triangle = new uint[3];
            triangle[2] = triangles[trianglePointIter];
            triangle[1] = triangles[trianglePointIter + 1];
            triangle[0] = triangles[trianglePointIter + 2];
            meshTriangleMsgs[meshTriangleIdx] = new MeshTriangleMsg(triangle);

            trianglePointIter += 3;
        }

        return new MeshMsg(meshTriangleMsgs, vertexPoints);
    }

    public static PoseMsg PoseInRobotFrameMsg(GameObject robot, GameObject gameObject)
    {
        Vector3<FLU> positionInRobotFrame;

        Quaternion<FLU> orientationInRobotFrame;

        Matrix4x4 worldToRobotTransformMat;

        worldToRobotTransformMat = robot.transform.worldToLocalMatrix;
        positionInRobotFrame = (worldToRobotTransformMat.MultiplyVector(gameObject.transform.position - robot.transform.position)).To<FLU>();
        orientationInRobotFrame = (worldToRobotTransformMat.rotation * Quaternion.Euler(gameObject.transform.eulerAngles.x, gameObject.transform.eulerAngles.y, gameObject.transform.eulerAngles.z)).To<FLU>();

        PoseMsg poseMsg = new PoseMsg()
        {
            position = new PointMsg(positionInRobotFrame.x, positionInRobotFrame.y, positionInRobotFrame.z),
            orientation = new QuaternionMsg(orientationInRobotFrame.x, orientationInRobotFrame.y, orientationInRobotFrame.z, orientationInRobotFrame.w)
        };

        return poseMsg;
    }

}

