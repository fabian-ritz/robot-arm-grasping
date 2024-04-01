using System;
using System.Collections.Generic;
using TMPro;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.SideChannels;
using UnityEngine;
using Random = UnityEngine.Random;

public class LiftAgent5D : Agent
{    
    StringLogSideChannel stringChannel;

    #region Text & GUI Elements
    [Header("Text & GUI")]
    public TextMeshProUGUI rewardText;
    public TextMeshProUGUI episodeInfoText;
    #endregion

    #region Target Object & Target Position
    [Header("Target Object & Position")]
    public GameObject target;
    public GameObject targetPosition;
    public float targetPositionThreshold = 0.1f;
    public float grippingRadiusMin = 1.5f;
    public float grippingRadiusMax = 4.2f;
    #endregion

    #region Gripper Parameters
    [Header("Gripper Heuristik")]
    public float gripperCloseThreshold = 0.018f;
    public float gripperOpenThreshold = 0.024f;
    public Color gripperClosedColor = Color.green;
    public int maxAllowedCollisions = 0;
    #endregion

    private Rigidbody targetBody;
    private ArticulationBody[] articulationChain;
    private RobotController robotController;
    private AutomatedGripper automatedGripper;

    private bool goalReached;
    private bool targetOutOfBounds;
    private bool tooManyCollisions;
    private bool newCollision;
    private int totalCollisions;
    private float distanceToTarget;
    private float distanceToGoal;
    private float lastScore;

    // temp variables for observation
    private Vector3 targetVectorObs;
    private float targetDistanceObs;
    private Quaternion targetRotationObs;

    // All Robot Arm Joints (index: joint_name, joint type)
    // 0: base,         fixed
    // 1: shoulder,     revolute
    // 2: arm,          revolute
    // 3: elbow,        revolute
    // 4: forearm,      revolute
    // 5: wrist,        revolute
    // 6: hand,         revolute
    // 7: tool,         fixed
    // 8: gripper_base, fixed
    // 9: servo_head,   fixed
    // 10: rod_left,    fixed
    // 11: gripperLeft, prismatic
    // 12: rod_right,   fixed
    // 13: gripperRight, prismatic
    private readonly List<int> movableJointIndices = new() {1,2,3,4,5};

    void Awake()
    {
        // We create the Side Channel
        stringChannel = new StringLogSideChannel();
        // When a Debug.Log message is created, we send it to the stringChannel
        Application.logMessageReceived += stringChannel.SendDebugStatementToPython;
        // The channel must be registered with the SideChannelManager class
        SideChannelManager.RegisterSideChannel(stringChannel);

        articulationChain = this.GetComponentsInChildren<ArticulationBody>();
        targetBody = target.GetComponent<Rigidbody>();

        robotController = (RobotController)this.GetComponentInParent(typeof(RobotController));
        if (robotController == null)
        {
            throw new NullReferenceException("Could not find an instance of RobotController");
        }

        automatedGripper = new(articulationChain, target, robotController, gripperCloseThreshold,
            gripperOpenThreshold, gripperClosedColor);

        Physics.simulationMode = SimulationMode.Script;  // decide manually when to step unities physics
    }

    protected void OnDestroy()
    {
        // De-register the Debug.Log callback
        Application.logMessageReceived -= stringChannel.SendDebugStatementToPython;
        if (Academy.IsInitialized){
            SideChannelManager.UnregisterSideChannel(stringChannel);
        }
    }

    public void FixedUpdate()
    {
        // collect observation and request an action from the (RL) policy for arm positioning and send it to the RobotController
        RequestDecision();
        // request action from the grupper heuristic (open / close) and send it to the RobotController
        automatedGripper.UpdateGripperState();
        // calculate the actual xDrive commands w.r.t. desired actions and physical limits
        robotController.CalculateNextMotion();

        // perform one step of the physics simulation (amongst others, the xDrives perform the calculated motion)
        Physics.Simulate(Time.fixedDeltaTime);

        // check whether collisions occured
        UpdateCollisionCounter();
        // update distance metrics
        UpdateDistances();
        // check whether the goal is reached
        GoalReachedCheck();
        // check whether the target object is reachable
        TargetOutOfBoundsCheck();
        
        // calculate rewards
        SetReward(CalculateStepReward());
        
        // check for episode end
        if ( this.StepCount == this.MaxStep -1 || targetOutOfBounds || goalReached || tooManyCollisions)
        {
            SendEpisodeMetrics();
            if (targetOutOfBounds || goalReached || tooManyCollisions)
            {
                EndEpisode();
            }
        }
    }

    public void Update()
    {
        SetText(rewardText, "Score: " + lastScore.ToString("F2"));
        SetText(episodeInfoText,  "Episode: " + this.CompletedEpisodes.ToString() + "\nStep: " + this.StepCount + " / " + this.MaxStep);
    }

    private void SetText(TextMeshProUGUI textElement, string newText)
    {
        textElement.text = newText;
    }

    public override void OnEpisodeBegin()
    {
        automatedGripper.Reset();
        ResetJoints();
        ResetTarget();
        ResetMetrics();
    }

    private void UpdateCollisionCounter()
    {
        if (newCollision)
        {
            totalCollisions += 1;
            newCollision = false;
            if (totalCollisions > maxAllowedCollisions)
            {
                tooManyCollisions = true;
            }
        }
    }

    private void GoalReachedCheck()
    {
        if (distanceToGoal < targetPositionThreshold)
        {
            goalReached = true;
        }
    }

    private void TargetOutOfBoundsCheck()
    {
        bool isOutOfBoundsHeight = target.transform.position.y < 6.0 || target.transform.position.y > 10;
        if (TargetOutOfGrippingRadius() || isOutOfBoundsHeight)
        {
            targetOutOfBounds = true;
        }
    }

    private bool TargetOutOfGrippingRadius()
    {
        float distanceFromRobot = Utilities.GetCircularDistance(transform.root, target.transform);
        return (distanceFromRobot < grippingRadiusMin || distanceFromRobot > grippingRadiusMax);
    }

    private void SendEpisodeMetrics()
    {
        string episodeResults = Convert.ToInt32(goalReached).ToString()
            + ";" + Convert.ToInt32(targetOutOfBounds).ToString()
            + ";" + lastScore.ToString()
            + ";" + automatedGripper.GetGripCounter().ToString()
            + ";" + totalCollisions.ToString()
            + ";" + distanceToTarget.ToString()
            + ";" + distanceToGoal.ToString();
        stringChannel.SendMessageToPython(episodeResults);
    }

    private void ResetJoints()
    {
        foreach (ArticulationBody joint in articulationChain)
        {
            joint.jointPosition = new ArticulationReducedSpace(0f, 0f, 0f);
            joint.jointForce = new ArticulationReducedSpace(0f, 0f, 0f);
            joint.jointVelocity = new ArticulationReducedSpace(0f, 0f, 0f);
            joint.SetDriveTarget(ArticulationDriveAxis.X, 0);
        }
    }

    private void ResetMetrics()
    {
        goalReached = false;
        tooManyCollisions = false;
        newCollision = false;
        totalCollisions = 0;
        UpdateDistances();
        lastScore = -distanceToGoal - distanceToTarget;
    }

    private void UpdateDistances()
    {
        distanceToTarget = (target.transform.position - automatedGripper.GetCenterPosition()).sqrMagnitude;
        distanceToGoal = (targetPosition.transform.position - target.transform.position).sqrMagnitude;

        if (!automatedGripper.SuccessfullGrip())
        {
            targetVectorObs = target.transform.position - automatedGripper.GetCenterPosition();
            targetDistanceObs = distanceToTarget;
            targetRotationObs = Utilities.GetRotationDifference(automatedGripper.GetCenterRotation(), target.transform.rotation);
        }
        else
        {
            targetVectorObs = targetPosition.transform.position - automatedGripper.GetCenterPosition();
            targetDistanceObs = distanceToGoal;
            targetRotationObs = Utilities.GetRotationDifference(automatedGripper.GetCenterRotation(), targetPosition.transform.rotation);
        }
    }

    public void HandleCollision(string detectedBy)
    {
        newCollision = true;
        if (detectedBy == "DummyValue")
        {
            Debug.Log("Robot: Hit message received by " + detectedBy);
        }
    }

    private float CalculateStepReward()
    {
        float currentScore = -distanceToTarget - distanceToGoal - ((float)totalCollisions * 5.0f) - ((float)StepCount * 0.01f);
        if (automatedGripper.SuccessfullGrip())
        {
            currentScore += 5.0f;
            if (goalReached)
            {
                currentScore += 10.0f;
            }
        }
        float stepReward = currentScore - lastScore;
        lastScore = currentScore;
        return stepReward;
    }

    private void ResetTarget()
    {
        // reset kinetic energy of target
        targetBody.velocity = new Vector3(0f, 0f, 0f);
        // get random target position ...
        target.transform.position = Utilities.GetRandomPosition(transform.root.position.x, transform.root.position.z, 2.0f, 3.7f, 0.2f, 0.8f);
        // ... and rotation
        target.transform.rotation = Quaternion.Euler(0f, Random.Range(-180f, 180f), 0f);
        // place target position above target
        targetPosition.transform.position = new Vector3(target.transform.position.x, target.transform.position.y + 1.0f, target.transform.position.z);
        // reset metric
        targetOutOfBounds = false;
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        List<float> observation = new();

        // 5 movable (revolute) joints, each with...
        foreach (int movableJoint in movableJointIndices){
            // ... the 1d scaled joint position and ...
            float jointPos = articulationChain[movableJoint].jointPosition[0];
            float posMin = articulationChain[movableJoint].xDrive.lowerLimit * Mathf.Deg2Rad;
            float posMax = articulationChain[movableJoint].xDrive.upperLimit * Mathf.Deg2Rad;
            observation.Add(Utilities.MapToInterval(jointPos, posMin, posMax, -1, 1)); // [?,?] -> [-1;1])
            // ... the 1d scaled joint velocity
            float jointVel = articulationChain[movableJoint].jointVelocity[0];
            observation.Add(Mathf.Clamp(Utilities.MapToInterval(jointVel, -0.5f, 0.5f, -1, 1), -1f, 1f)); // [-0.5,0.5] -> [-1;1])
        }

        // 3d rotation difference between gripper center current target
        Vector3 rotationDiff = Utilities.AsNormalizedEulerAngles(targetRotationObs); // 4D Quaterion -> 3D [-1;1]
        observation.Add(rotationDiff.x);
        observation.Add(rotationDiff.y);
        observation.Add(rotationDiff.z);

        // 3d normalized vector pointing to current target
        Vector3 vecToTarget = Vector3.Normalize(targetVectorObs);
        observation.Add(vecToTarget.x);
        observation.Add(vecToTarget.y);
        observation.Add(vecToTarget.z);

        // 1d non-linearly scaled distance to current target
        observation.Add(Utilities.MapToInterval(Mathf.Sqrt(targetDistanceObs), 0, 8f, 0, 1)); // [0;60] -> [0;8] -> [0;1]

        // 1d left gripper force
        observation.Add(Mathf.Clamp(Utilities.MapToInterval(automatedGripper.GetLeftGripperForce(), 0, 200f, 0, 1), 0, 1));
        
        // 1d right gripper force
        observation.Add(Mathf.Clamp(Utilities.MapToInterval(automatedGripper.GetRightGripperForce(), 0, 200f, 0, 1), 0, 1));

        // 1d gripperClosed indicator
        observation.Add(Convert.ToInt32(automatedGripper.SuccessfullGrip()));

        sensor.AddObservation(observation);
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        // one action per movable joint
        for (int actionIndex = 0; actionIndex < movableJointIndices.Count; actionIndex++)
        {
            // get index of movable joint within articulationChain
            int movableJointIndex = movableJointIndices[actionIndex];
            // clamp the action respective action (just in case...)
            float clampedAction = Mathf.Clamp(actions.ContinuousActions[actionIndex], -1f, 1f);
            // scale the respective action
            float posMin = articulationChain[movableJointIndex].xDrive.lowerLimit * Mathf.Deg2Rad;
            float posMax = articulationChain[movableJointIndex].xDrive.upperLimit * Mathf.Deg2Rad;
            float scaledAction = Utilities.MapToInterval(clampedAction, -1, 1, posMin, posMax);
            // set motion target for the joint
            robotController.SetMotionTarget(movableJointIndex, scaledAction);
        }
    }
}