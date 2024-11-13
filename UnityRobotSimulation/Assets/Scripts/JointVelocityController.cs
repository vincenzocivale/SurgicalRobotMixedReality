using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.UrdfImporter;
using RosMessageTypes.Std;

public class JointVelocityController : MonoBehaviour
{
    [Header("ROS Topic Settings")]
    [Tooltip("Nome del topic ROS da cui ricevere i messaggi di velocit�")]
    [SerializeField] private string topicName;

    [Header("Target Settings")]
    [Tooltip("Il GameObject che contiene il joint da controllare")]
    [SerializeField] private GameObject targetGO;

    [Tooltip("Fattore di scala per la velocit�")]
    [SerializeField] private float scaleFactor = 0.01f;

    private UrdfJoint joint;  // Riferimento al joint URDF
    private float currentSpeed = 0f;  // Velocit� corrente ricevuta da ROS

    private void Start()
    {
        if (!InitializeJoint())
        {
            Debug.LogError("Impossibile inizializzare il joint o topicName non impostato.");
            enabled = false;
            return;
        }

        // Iscrizione al topic ROS
        ROSConnection.GetOrCreateInstance().Subscribe<Float64Msg>(topicName, UpdateSpeedFromROS);
    }

    /// <summary>
    /// Inizializza il componente joint e verifica la correttezza dei parametri.
    /// </summary>
    /// <returns>True se l'inizializzazione ha avuto successo, altrimenti false</returns>
    private bool InitializeJoint()
    {
        if (string.IsNullOrEmpty(topicName))
        {
            Debug.LogError("Il topicName non � impostato.");
            return false;
        }

        if (targetGO == null)
        {
            Debug.LogError("targetGO non � assegnato. Assicurati di assegnare un GameObject.");
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
    /// Callback per aggiornare la velocit� corrente ricevuta dal topic ROS.
    /// </summary>
    /// <param name="message">Messaggio Float64 ricevuto da ROS</param>
    private void UpdateSpeedFromROS(Float64Msg message)
    {
        currentSpeed = (float)message.data;
        ApplyJointMovement();
    }

    /// <summary>
    /// Applica il movimento al joint in base alla velocit� corrente e al fattore di scala.
    /// </summary>
    private void ApplyJointMovement()
    {
        float delta = currentSpeed * scaleFactor * Time.fixedDeltaTime;
        joint.UpdateJointState(delta);
    }
}
