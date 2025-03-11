using UnityEngine;
using UnityEngine.InputSystem;

public class InputHandler : MonoBehaviour
{
    [Header("Input Action Assets")]
    [SerializeField] private InputActionAsset input;
    
    [Header("Action map name references")]
    [SerializeField] private string actionMapName = "Player";
    
    [Header("Action name references")]
    [SerializeField] private string move = "Move";
    
    private InputAction moveAction;
    
    public Vector2 MoveInput { get; private set; }
    
    public static InputHandler Instance { get; private set; }
    private void Awake()
    {
        if (Instance == null)
        {
            Instance = this;
            DontDestroyOnLoad(gameObject);
        }
        else
        {
            Debug.LogWarning("InputHandler instance already exists. Destroying this instance.");
            Destroy(gameObject);
        }
        
        moveAction = input.FindActionMap(actionMapName).FindAction(move);
        RegisterInputActions();
    }

    private void RegisterInputActions()
    {
        moveAction.performed += context => MoveInput = context.ReadValue<Vector2>();
        moveAction.canceled += context => MoveInput = Vector2.zero;
    }

    private void OnEnable()
    {
        moveAction.Enable();
    }
    
    private void OnDisable()
    {
         moveAction.Disable();
    }
}
