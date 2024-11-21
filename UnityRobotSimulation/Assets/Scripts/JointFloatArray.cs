using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.UrdfImporter;
using static Unity.Robotics.UrdfImporter.UrdfJoint;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;

public class JointFloatArray : MonoBehaviour
{
    [Tooltip("Nome del topic ROS da cui ricevere i messaggi di velocità")]
    [SerializeField] private string topicName;

    [Tooltip("Lista dei GameObject target da controllare")]
    [SerializeField] private List<GameObject> targetGameObjects;

    void Start()
    {
        ROSConnection.GetOrCreateInstance().Subscribe<Float64MultiArrayMsg>(topicName, UpdateJoints);

    }

    public void UpdateJoints(Float64MultiArrayMsg jointFloatArray)
    {
        for (int i = 0; i < targetGameObjects.Count; i++)
        {
            UrdfJoint currentJoint = targetGameObjects[i].GetComponent<UrdfJoint>();
            float jointValue = (float)jointFloatArray.data[i];
            float delta = jointValue - currentJoint.GetPosition();
            currentJoint.UpdateJointState(delta);
        }
    }
}
