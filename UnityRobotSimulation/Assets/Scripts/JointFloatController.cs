using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.UrdfImporter;
using RosMessageTypes.Std;

public class JointFloatController : MonoBehaviour
{
    [Header("ROS Topic Settings")]
    [Tooltip("Nome del topic ROS da cui ricevere i messaggi di velocità")]
    [SerializeField] private string topicName;

    [Header("Target Settings")]
    [Tooltip("Il GameObject che contiene il joint da controllare")]
    [SerializeField] private GameObject targetGO;

    private UrdfJoint joint;  // Riferimento al joint URDF

    private void Start()
    {
        if (!InitializeJoint())
        {
            Debug.LogError("Impossibile inizializzare il joint o topicName non impostato.");
            enabled = false;
            return;
        }

        // Iscrizione al topic ROS
        ROSConnection.GetOrCreateInstance().Subscribe<Float64Msg>(topicName, UpdateJointValue);
    }

    /// <summary>
    /// Inizializza il componente joint e verifica la correttezza dei parametri.
    /// </summary>
    /// <returns>True se l'inizializzazione ha avuto successo, altrimenti false</returns>
    private bool InitializeJoint()
    {
        if (string.IsNullOrEmpty(topicName))
        {
            Debug.LogError("Il topicName non è impostato.");
            return false;
        }

        if (targetGO == null)
        {
            Debug.LogError("targetGO non è assegnato. Assicurati di assegnare un GameObject.");
            return false;
        }

        joint = targetGO.GetComponent<UrdfJoint>();
        if (joint == null)
        {
            Debug.LogError("Nessun componente UrdfJoint trovato sul GameObject target.");
            return false;
        }

        return true;
    }

    /// <summary>
    /// Callback per aggiornare la velocità corrente ricevuta dal topic ROS.
    /// </summary>
    /// <param name="message">Messaggio Float64 ricevuto da ROS</param>
    private void UpdateJointValue(Float64Msg message)
    {
        float jointValue = (float)message.data;
        float delta = jointValue - joint.GetPosition();
        joint.UpdateJointState(delta);

        UnityEngine.Debug.Log("Joint " + joint.name + " value updated to: " + jointValue);
    }

    
}
