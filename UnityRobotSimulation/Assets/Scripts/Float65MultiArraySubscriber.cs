using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.UrdfImporter;
using static Unity.Robotics.UrdfImporter.UrdfJoint;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;


public class FloatArraySubscriber : MonoBehaviour
{
    public GameObject rootJointGO;
    /*public List<float> jointStates;*/
    [SerializeField]
    private string jointTopic;

    private List<UrdfJoint> urdfJoints = new List<UrdfJoint>();
    private bool updateJointStates = true;

    void Start()
    {

        if (rootJointGO == null || jointTopic == null)
        {
            Debug.LogError("Assicurati di aver assegnato rootJointGO e jointStates con valori appropriati.");
            return;
        }

        UrdfJoint currentJoint = rootJointGO.GetComponent<UrdfJoint>();

        if (currentJoint != null)
        {
            urdfJoints.Add(currentJoint);
        }
        else
        {
            Debug.LogError("Il GameObject rootJointGO non contiene un componente UrdfJoint.");
            return;
        }

        GameObject childJointGO = FindChildJoint(rootJointGO);
        while (childJointGO != null)
        {
            currentJoint = childJointGO.GetComponent<UrdfJoint>();

            // Si presume che i Joint Fixed non vengano pubblicati
            if (currentJoint != null && currentJoint.JointType != JointTypes.Fixed)
            {
                urdfJoints.Add(currentJoint);
            }

            childJointGO = FindChildJoint(childJointGO);
        }

        ROSConnection.GetOrCreateInstance().Subscribe<Float64MultiArrayMsg>(jointTopic, UpdateJointStates);

    }


    public void UpdateJointStates(Float64MultiArrayMsg jointTrajectory)
    {
        
        var jointsValues = jointTrajectory.data;  // Prende le posizioni dal primo punto della traiettoria

        for (int i = 0; i < urdfJoints.Count && i < jointsValues.Length; i++)
        {
            UrdfJoint currentJoint = urdfJoints[i];
            float jointValue = (float)jointsValues[i];  // Prende la posizione del joint corrente
            float delta = jointValue - currentJoint.GetPosition();
            currentJoint.UpdateJointState(delta);

            Debug.Log("Joint " + currentJoint.name + " set to position " + jointValue);
        }
        
    }

    public static GameObject FindChildJoint(GameObject parent)
    {
        foreach (Transform child in parent.transform)
        {
            if (child.GetComponent<UrdfJoint>() != null)
            {
                return child.gameObject;
            }
        }
        return null;
    }
}
