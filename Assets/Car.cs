using UnityEngine;

public class Car : MonoBehaviour
{
    [Header("Tire Configuration")]
    [SerializeField] private GameObject[] tires;
    [SerializeField] private int[] steeringTireIndices = { 0, 1 };
    [SerializeField] private bool[] isFrontTire;
    [SerializeField] private float tireMass = 20f;
    [SerializeField] private float maxSteeringAngle = 30f;
    [SerializeField] private float frontTireTractionPercent = 0.05f;
    [SerializeField] private float rearTireTractionPercent = 0.3f;

    [Header("Suspension")]
    [SerializeField] private float suspensionLength = 1f;
    [SerializeField] private float suspensionRestDist = 0.5f;
    [SerializeField] private float springStrength = 10000f;
    [SerializeField] private float springDamper = 1000f;
    
    [Header("Driving Dynamics")]
    [SerializeField] private float carTopSpeed = 30f;
    [SerializeField] private float carTorque = 500f;
    [SerializeField] private AnimationCurve powerCurve;
    [SerializeField] private AnimationCurve frontTireTractionCurve;
    [SerializeField] private AnimationCurve rearTireTractionCurve;
    
    [Header("Visual Debug")]
    [SerializeField] private bool showDebugLines = true;

    private Rigidbody carRigidBody;
    private Transform carTransform;
    private InputHandler input;
    
    // Cached vectors to avoid garbage collection
    private static readonly Vector3 DownDirection = Vector3.down;
    private readonly RaycastHit[] tireRayResults = new RaycastHit[1];

    private void Start()
    {
        carRigidBody = GetComponent<Rigidbody>();
        carTransform = transform;
        input = InputHandler.Instance;
    }

    private void FixedUpdate()
    {
        ApplySteering(input.MoveInput.x);
        ProcessTirePhysics();
    }

    private void ProcessTirePhysics()
    {
        for (int i = 0; i < tires.Length; i++)
        {
            Transform tireTransform = tires[i].transform;
            
            // Raycast to the ground (non-alloc version to reduce GC)
            int hitCount = Physics.RaycastNonAlloc(tireTransform.position, -tireTransform.up, tireRayResults, suspensionLength);
            if (hitCount == 0)
                continue;

            var tireRay = tireRayResults[0];
            
            // Apply suspension force
            ApplySuspensionForce(tireTransform, tireRay);
            
            // Apply lateral traction force
            ApplyLateralTractionForce(tireTransform, i);
            
            // Apply acceleration/braking force
            ApplyAccelerationForce(tireTransform, input.MoveInput.y);
        }
    }

    private void ApplySuspensionForce(Transform tireTransform, RaycastHit tireRay)
    {
        Vector3 springDir = tireTransform.up;
        
        // Spring physics calculations
        Vector3 tireWorldVel = carRigidBody.GetPointVelocity(tireTransform.position);
        float offset = suspensionRestDist - tireRay.distance;
        float velocityAlongSpring = Vector3.Dot(springDir, tireWorldVel);
        float springForce = (offset * springStrength) - (velocityAlongSpring * springDamper);
        
        // Apply force
        carRigidBody.AddForceAtPosition(springDir * springForce, tireTransform.position);
        
        // Debug visualization
        if (showDebugLines)
            DrawDebugVector(tireTransform.position, springDir * springForce, Color.green);
    }

    private void ApplyLateralTractionForce(Transform tireTransform, int tireIndex)
    {
        Vector3 steeringDir = tireTransform.right;
        Vector3 tireWorldVel = carRigidBody.GetPointVelocity(tireTransform.position);
        float lateralVelocity = Vector3.Dot(steeringDir, tireWorldVel);
        
        // Calculate slip and traction
        float slipPercentage = Mathf.Clamp01(Mathf.Abs(lateralVelocity) / carTopSpeed);
        float tractionFactor = isFrontTire[tireIndex] 
            ? frontTireTractionPercent 
            : rearTireTractionPercent;
        
        // Apply counter-force
        float desiredVelChange = -lateralVelocity * tractionFactor;
        float desiredAccel = desiredVelChange / Time.fixedDeltaTime;
        Vector3 tractionForce = steeringDir * (tireMass * desiredAccel);
        
        carRigidBody.AddForceAtPosition(tractionForce, tireTransform.position);
        
        // Debug visualization
        if (showDebugLines)
            DrawDebugVector(tireTransform.position, tractionForce, Color.red);
    }

    private void ApplyAccelerationForce(Transform tireTransform, float inputValue)
    {
        if (Mathf.Approximately(inputValue, 0f))
            return;
            
        // Calculate available torque based on speed curve
        Vector3 accelDir = tireTransform.forward;
        float carSpeed = Vector3.Dot(carTransform.forward, carRigidBody.linearVelocity);
        float normalizedSpeed = Mathf.Clamp01(Mathf.Abs(carSpeed) / carTopSpeed);
        float availableTorque = powerCurve.Evaluate(normalizedSpeed) * inputValue * carTorque;
        
        // Apply force
        carRigidBody.AddForceAtPosition(accelDir * availableTorque, tireTransform.position);
        
        // Debug visualization
        if (showDebugLines)
            DrawDebugVector(tireTransform.position, accelDir * availableTorque, Color.blue);
    }

    private void ApplySteering(float steerInput)
    {
        float steeringAngle = maxSteeringAngle * steerInput;
        
        foreach (int tireIndex in steeringTireIndices)
        {
            if (tireIndex < 0 || tireIndex >= tires.Length)
                continue;
                
            Transform tireTransform = tires[tireIndex].transform;
            tireTransform.localRotation = Quaternion.Euler(0, steeringAngle, 0);
        }
    }
    
    private void DrawDebugVector(Vector3 start, Vector3 force, Color color)
    {
        Debug.DrawRay(start, force, color);
        
        Vector3 arrowEnd = start + force;
        float arrowSize = Mathf.Max(0.1f, force.magnitude * 0.1f);
        Vector3 right = Vector3.Cross(force.normalized, Vector3.up).normalized;
        if (right.magnitude < 0.1f)
            right = Vector3.Cross(force.normalized, Vector3.forward).normalized;
            
        Debug.DrawRay(arrowEnd, -force.normalized * arrowSize + right * (arrowSize * 0.5f), color);
        Debug.DrawRay(arrowEnd, -force.normalized * arrowSize - right * (arrowSize * 0.5f), color);
    }
}