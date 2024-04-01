using UnityEngine;
using System;

public class DirectRobotControl : MonoBehaviour
{
    private ArticulationBody[] ownArticulationChain;
    private RobotController robotController;
    private AutomatedGripper automatedGripper;

    [Tooltip("Color to highlight closed grippers")]
    public float gripperCloseThreshold;
    public float gripperOpenThreshold;
    public Color gripperClosedColor = new(0.0f, 1.0f, 0.0f, 1.0f);

    [Tooltip("The object the gripper will react to")]
    public GameObject target;
    public GameObject targetPosition;
    private Rigidbody targetBody;

    // Start is called before the first frame update
    void Start()
    {
        robotController = (RobotController)this.GetComponentInParent(typeof(RobotController));
        if (robotController == null)
        {
            throw new NullReferenceException("Could not find an instance of RobotController");
        }

        ownArticulationChain = this.GetComponentsInChildren<ArticulationBody>();
        targetBody = target.GetComponent<Rigidbody>();

        automatedGripper = new(ownArticulationChain, target, robotController, gripperCloseThreshold,
            gripperOpenThreshold, gripperClosedColor);
    }

    void Update()
    {
        // use to print any information you would like to debug
        // PrintDebugMessage();
    }

    void PrintDebugMessage()
    {
        ArticulationReducedSpace jointPos = ownArticulationChain[1].jointPosition;
        Debug.Log(jointPos[0].ToString());
    }

    public void Mimic(ArticulationBody[] otherArticulationChain)
    {
        if(ownArticulationChain.Length != otherArticulationChain.Length)
        {
            return;
        }

        for(int j = 0; j < ownArticulationChain.Length; j++)
        {
            ownArticulationChain[j].jointPosition = otherArticulationChain[j].jointPosition;
        }
        automatedGripper.UpdateGripperState();  // depending on target distance, open or close gripper
        robotController.CalculateNextMotion();
    }

    public void ResetMe(Vector3 newTargetPosition, Quaternion newTargetRotation)
    {
        automatedGripper.Reset();
        // reset kinetic energy of target
        targetBody.velocity = new Vector3(0f, 0f, 0f);
        // set target position and rotation
        target.transform.SetPositionAndRotation(new Vector3(newTargetPosition.x - 4, newTargetPosition.y, newTargetPosition.z), newTargetRotation);
        // place target position above target
        targetPosition.transform.position = new Vector3(target.transform.position.x, target.transform.position.y + 1.0f, target.transform.position.z);

    }
}
