using UnityEngine;

public class RobotJointControl
{
    private readonly ArticulationBody joint;
    private readonly float speed; // Units: degree/s
    private readonly float torque; // Units: Nm or N
    private readonly float acceleration;// Units: m/s^2 / degree/s^2
    private float rotationTarget = 0f;

    public RobotJointControl(ArticulationBody joint, float speed, float torque, float acceleration)
    {
        this.joint = joint;
        this.speed = speed;
        this.torque = torque;
        this.acceleration = acceleration;
    }

    public void SetMotionTarget(float value)
    {
        rotationTarget = value;
    }

    public void CalculateNextMotion()
    {
        if (joint.jointType != ArticulationJointType.FixedJoint)
        {
            ArticulationDrive currentDrive = joint.xDrive;
            float newTargetDelta = rotationTarget * Time.fixedDeltaTime * speed;

            if (joint.jointType == ArticulationJointType.RevoluteJoint)
            {
                if (joint.twistLock == ArticulationDofLock.LimitedMotion)
                {
                    if (newTargetDelta + currentDrive.target > currentDrive.upperLimit)
                    {
                        currentDrive.target = currentDrive.upperLimit;
                    }
                    else if (newTargetDelta + currentDrive.target < currentDrive.lowerLimit)
                    {
                        currentDrive.target = currentDrive.lowerLimit;
                    }
                    else
                    {
                        currentDrive.target += newTargetDelta;
                    }
                }
                else
                {
                    currentDrive.target += newTargetDelta;
                }
            }
            else if (joint.jointType == ArticulationJointType.PrismaticJoint)
            {
                if (joint.linearLockX == ArticulationDofLock.LimitedMotion)
                {
                    if (newTargetDelta + currentDrive.target > currentDrive.upperLimit)
                    {
                        currentDrive.target = currentDrive.upperLimit;
                    }
                    else if (newTargetDelta + currentDrive.target < currentDrive.lowerLimit)
                    {
                        currentDrive.target = currentDrive.lowerLimit;
                    }
                    else
                    {
                        currentDrive.target += newTargetDelta;
                    }
                }
                else
                {
                    currentDrive.target += newTargetDelta;
                }
            }
            joint.xDrive = currentDrive;
        }
    }
}
