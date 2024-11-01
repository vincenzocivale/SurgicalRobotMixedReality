using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.UrdfImporter;
using static Unity.Robotics.UrdfImporter.UrdfJoint;

public class RobotJointSubscriber : MonoBehaviour
{
    public GameObject rootJointGO;
    public List<float> jointStates;
    private List<UrdfJoint> urdfJoints = new List<UrdfJoint>();
    private bool updateJointStates = true;

    void Start()
    {
        
        if (rootJointGO == null || jointStates == null || jointStates.Count == 0)
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

        
    }
    void Update()
    {
        if (updateJointStates)
        {
            UpdateJointStates();
        }
        updateJointStates = false;
    }
    public void UpdateJointStates()
    {
        for (int i = 0; i < jointStates.Count; i++)
        {
            UrdfJoint currentJoint = urdfJoints[i];
            currentJoint.UpdateJointState(jointStates[i]);
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
