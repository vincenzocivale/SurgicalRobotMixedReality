//using UnityEngine;
//using Unity.Robotics.ROSTCPConnector;
//using RosMessageTypes.Geometry;
//using static Unity.Robotics.UrdfImporter.UrdfJoint;
//using Unity.Robotics.UrdfImporter;

//class JointQuaternionController : MonoBehaviour
//{
//    public GameObject jointGO;

//    [SerializeField]
//    private string jointTopic;


//    private Quaternion targetRotation;
//    private Vector3 rotationAxis;

//    void Start()
//    {
//        if (jointGO == null)
//        {
//            Debug.LogError("Assicurati di aver assegnato jointGO.");
//            return;
//        }

//        UrdfJointRevolute urdfJointRevolute = jointGO.GetComponent<UrdfJointRevolute>();
        

//        ROSConnection.GetOrCreateInstance().Subscribe<QuaternionMsg>(jointTopic, UpdateJointQuaternion);
//    }

//    public void UpdateJointQuaternion(QuaternionMsg jointQuaternion)
//    {
//        Quaternion targetRotation = new Quaternion((float)jointQuaternion.x, (float)jointQuaternion.y, (float)jointQuaternion.z, (float)jointQuaternion.w);

//        Quaternion currentRotation = transform.rotation;

//        Quaternion deltaRotation = targetRotation * Quaternion.Inverse(currentRotation);

//        deltaRotation.ToAngleAxis(out float angle, out Vector3 axis);

//        // Proietta l'asse calcolato sull'asse del giunto
//        Vector3 jointAxis = urdfJointRevolute.axisofMotion.normalized;
//        float angleDelta = angle * Mathf.Sign(Vector3.Dot(axis, jointAxis));
//    }

//    float GetAxisAngle(Quaternion deltaRotation, Vector3 axis)
//    {
//        // Estrarre l'angolo di rotazione intorno all'asse
//        deltaRotation.ToAngleAxis(out float angle, out Vector3 deltaAxis);

//        // Determina il segno dell'angolo confrontando l'asse
//        float sign = Mathf.Sign(Vector3.Dot(deltaAxis, axis));

//        // Restituisce l'angolo con il segno
//        return angle * sign;
//    }
//}

