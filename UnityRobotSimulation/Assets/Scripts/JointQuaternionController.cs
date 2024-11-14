using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;

class JointQuaternionController : MonoBehaviour
{
    public GameObject jointGO;

    [SerializeField]
    private string jointTopic;

    void Start()
    {
        if (jointGO == null)
        {
            Debug.LogError("Assicurati di aver assegnato jointGO.");
            return;
        }

        ROSConnection.GetOrCreateInstance().Subscribe<QuaternionMsg>(jointTopic, UpdateJointQuaternion);
    }

    public void UpdateJointQuaternion(QuaternionMsg jointQuaternion)
    {
        jointGO.transform.rotation = new Quaternion((float)jointQuaternion.x, (float)jointQuaternion.y, (float)jointQuaternion.z, (float)jointQuaternion.w);
    }
}

