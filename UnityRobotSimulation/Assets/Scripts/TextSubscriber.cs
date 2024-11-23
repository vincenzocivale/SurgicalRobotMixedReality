using UnityEngine;
using TMPro;
using RosMessageTypes.Std; // Namespace ROS (assicurati di usare il corretto tipo di messaggi ROS)
using Unity.Robotics.ROSTCPConnector; // ROS-TCP-Connector per Unity

public class TextSubscriber : MonoBehaviour
{
    [Tooltip("Nome del topic ROS da sottoscrivere.")]
    public string rosTopicName;

    [Tooltip("Il GameObject di riferimento. Deve contenere o avere un figlio con un componente TextMeshPro.")]
    public GameObject targetGameObject;

    
    private string fixedText;

    private ROSConnection rosConnection;
    private TextMeshPro textMeshPro;

    void Start()
    {

        // Cerca il componente TextMeshPro nel GameObject o nei suoi figli
        if (targetGameObject != null)
        {
            textMeshPro = targetGameObject.GetComponentInChildren<TextMeshPro>();
            if (textMeshPro == null)
            {
                Debug.LogError("Nessun componente TextMeshPro trovato nel GameObject o nei suoi figli.");
            }

            fixedText = textMeshPro.text;

            // Configura la connessione ROS
            rosConnection = ROSConnection.GetOrCreateInstance();
            rosConnection.Subscribe<StringMsg>(rosTopicName, UpdateTextFromRos);
        }
        else
        {
            Debug.LogError("Target GameObject non assegnato!");
        }
    }

    /// <summary>
    /// Aggiorna il testo in base al messaggio ricevuto dal topic ROS.
    /// </summary>
    /// <param name="message">Messaggio ricevuto dal topic ROS.</param>
    private void UpdateTextFromRos(StringMsg message)
    {
            textMeshPro.text = fixedText + message.data;       
    }
}
