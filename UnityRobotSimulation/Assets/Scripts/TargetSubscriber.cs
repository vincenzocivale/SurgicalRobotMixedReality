using UnityEngine;
using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;
using Unity.Robotics.UrdfImporter;
using static Unity.Robotics.UrdfImporter.UrdfJoint;

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
        targetGO.transform.position = new Vector3(-(float)jointTrajectory.data[1], (float)jointTrajectory.data[2], (float)jointTrajectory.data[0]);
        UnityEngine.Debug.Log("Posizione aggiornata: "+ targetGO.transform.position);
    }
}
