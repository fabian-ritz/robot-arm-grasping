using UnityEngine;
using System.Collections.Generic;

public enum RotationDirection { None = 0, Positive = 1, Negative = -1 };

public class RobotController : MonoBehaviour
{

    #region Joints
    [Header("Joint Physics")]
    public float jointFriction;
    public float jointDamping;
    #endregion

    #region xDrives
    [Header("xDrives Physics")]
    public float xDriveStiffness;
    public float xDriveDamping;
    public float xDriveForceLimit;
    #endregion

    #region controllers
    [Header("Joint Controller Limits")]
    public float controllerSpeed;
    public float controllerTorque;
    public float controllerAcceleration;
    #endregion

    private ArticulationBody[] articulationChain;
    private List<RobotJointControl> robotJointController;

    void Start()
    {
        articulationChain = this.GetComponentsInChildren<ArticulationBody>();
        robotJointController = new();

        foreach (ArticulationBody joint in articulationChain)
        {
            // configure missing (not defined in imported NiryoOne) ArticulationJoint values
            joint.jointFriction = jointFriction;
            joint.angularDamping = jointDamping;
            // configure xDrive properties (unity way of controlling joints on x-Axis)
            ArticulationDrive drive = joint.xDrive;
            drive.stiffness = xDriveStiffness;
            drive.damping = xDriveDamping;
            drive.forceLimit = xDriveForceLimit;
            joint.xDrive = drive;
            // add custom controller (imported from URDF, uses xDrive.setTarget())
            RobotJointControl jointControl = new (joint, controllerSpeed, controllerTorque, controllerAcceleration);
            robotJointController.Add(jointControl);
        }
    }

    public void SetMotionDirection(int jointIndex, RotationDirection direction)
    {
        if (jointIndex < 0 || jointIndex >= articulationChain.Length)
        {
            return;
        }
        robotJointController[jointIndex].SetMotionTarget((float)direction);
    }

    public void SetMotionTarget(int jointIndex, float value)
    {
        if (jointIndex < 0 || jointIndex >= articulationChain.Length)
        {
            return;
        }
        robotJointController[jointIndex].SetMotionTarget(value);
    }

    public void CalculateNextMotion()
    {
        foreach(RobotJointControl jc in robotJointController)
        {
            jc.CalculateNextMotion();
        }
    }

}