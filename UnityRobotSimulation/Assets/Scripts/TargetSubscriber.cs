using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;

public class TargetSubscriber : MonoBehaviour
{
    public GameObject targetGO;

    [SerializeField]
    private string jointTopic;

    void Start()
    {

        if (targetGO == null)
        {
            Debug.LogError("Assicurati di aver assegnato targetGO.");
            return;
        }

        ROSConnection.GetOrCreateInstance().Subscribe<Float64MultiArrayMsg>(jointTopic, UpdateTargetTransform);

    }


    public void UpdateTargetTransform(Float64MultiArrayMsg jointTrajectory)
    {
        if (jointTrajectory.data.Length != 7)
        {
            Debug.LogError("Il messaggio ricevuto non contiene 7 elementi.");
            return;
        }

        targetGO.transform.position = new Vector3((float)jointTrajectory.data[0], (float)jointTrajectory.data[1], (float)jointTrajectory.data[2]);
        targetGO.transform.rotation = new Quaternion((float)jointTrajectory.data[3], (float)jointTrajectory.data[4], (float)jointTrajectory.data[5], (float)jointTrajectory.data[6]);
    }
}
