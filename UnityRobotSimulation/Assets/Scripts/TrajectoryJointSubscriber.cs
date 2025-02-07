using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.UrdfImporter;
using static Unity.Robotics.UrdfImporter.UrdfJoint;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Trajectory;
using System.Linq;

public enum MotionType
{
    Positions,
    Velocities
}


public class TrajectoryJointSubscriber : MonoBehaviour
{
    public GameObject rootJointGO;
    public MotionType motionType;
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

        ROSConnection.GetOrCreateInstance().Subscribe<JointTrajectoryMsg>(jointTopic, UpdateJointStates);

    }


    public void UpdateJointStates(JointTrajectoryMsg jointTrajectory)
    {
        if (jointTrajectory.points.Length > 0)
        {
            if (motionType == MotionType.Positions)
            {
                var positions = jointTrajectory.points[0].positions;  // Prende le posizioni dal primo punto della traiettoria

                for (int i = 0; i < urdfJoints.Count && i < positions.Length; i++)
                {
                    UrdfJoint currentJoint = urdfJoints[i];
                    float jointValue = (float)positions[i];  // Prende la posizione del joint corrente
                    float delta = jointValue - currentJoint.GetPosition();
                    currentJoint.UpdateJointState(delta);

                    Debug.Log("Joint " + currentJoint.name + " set to position " + jointValue);
                }
            }
            else if (motionType == MotionType.Velocities)
            {
                var velocities = jointTrajectory.points[0].velocities;  // Prende le velocità dal primo punto della traiettoria

                for (int i = 0; i < urdfJoints.Count && i < velocities.Length; i++)
                {
                    UrdfJoint currentJoint = urdfJoints[i];
                    float jointValue = (float)velocities[i];  // Prende la velocità del joint corrente
                    float delta = jointValue - currentJoint.GetVelocity();
                    currentJoint.UpdateJointState(delta);

                    Debug.Log("Joint " + currentJoint.name + " set to velocity " + jointValue);
                }
            }
            
        }

            
        else
        {
            Debug.LogWarning("Il messaggio JointTrajectory non contiene punti.");
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
