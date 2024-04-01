using UnityEngine;

public class AutomatedGripper
{
    private readonly GameObject target;
    private readonly RobotController robotController;
    private readonly ArticulationBody[] articulationChain;
    private readonly GameObject center;

    private readonly float gripperCloseThreshold;
    private readonly float gripperOpenThreshold;
    private Color gripperClosedColor;
    private Color[] defaultColors;

    private const int gripperBaseIndex = 8;
    private const int leftGripperIndex = 11;
    private const int rightGripperIndex = 13;
    private const float jointPosThreshold = 0.022f;

    private int grips = 0;
    private bool isClosing = false;
    private bool isOpening = false;

    public AutomatedGripper(
        ArticulationBody[] articulationChain,
        GameObject target,
        RobotController robotController,
        float gripperCloseThreshold,
        float gripperOpenThreshold,
        Color gripperClosedColor
        )
    {
        // set variables
        this.articulationChain = articulationChain;
        this.target = target;
        this.robotController = robotController;
        this.gripperCloseThreshold = gripperCloseThreshold;
        this.gripperOpenThreshold = gripperOpenThreshold;
        this.gripperClosedColor = gripperClosedColor;

        // set and visualize center of gripper
        center = SetupGripperCenter();

        // store default color of joints
        StoreJointColors(leftGripperIndex);
        StoreJointColors(rightGripperIndex);
    }

    public bool SuccessfullGrip()
    {
        return InClosedPosition();
    }

    public int GetGripCounter()
    {
        return grips;
    }

    public void Reset()
    {
        OpenGripper();
        isOpening = false;
        isClosing = false;
        grips = 0;
    }

    public Vector3 GetCenterPosition()
    {
        return center.transform.position;
    }

    public Quaternion GetCenterRotation()
    {
        return center.transform.rotation;
    }

    public float GetLeftGripperForce()
    {
        return Mathf.Abs(articulationChain[leftGripperIndex].driveForce[0]);
    }

    public float GetRightGripperForce()
    {
        return Mathf.Abs(articulationChain[rightGripperIndex].driveForce[0]);
    }

    private bool InClosedPosition()
    {
        return articulationChain[leftGripperIndex].jointPosition[0] < jointPosThreshold &&
            - articulationChain[rightGripperIndex].jointPosition[0] < jointPosThreshold;
    }

    private bool InOpenPosition()
    {
        return articulationChain[leftGripperIndex].jointPosition[0] > jointPosThreshold &&
            - articulationChain[rightGripperIndex].jointPosition[0] > jointPosThreshold;
    }

    public void UpdateGripperState()
    {
        if (!isOpening && !isClosing)
        {
            float distanceToTarget = (target.transform.position - center.transform.position).sqrMagnitude;

            // grip if the gripper is close enough to target object
            if (InOpenPosition() && distanceToTarget < gripperCloseThreshold)
            {
                CloseGripper();
                isClosing = true;
            }
            // reset the gripper if it is too far from the target object
            else if (InClosedPosition() && (distanceToTarget > gripperOpenThreshold))
            {
                OpenGripper();
                isOpening = true;
            }
        }
        else if (isOpening && InOpenPosition())
        {
            isOpening = false;
        }
        else if (isClosing && InClosedPosition())
        {
            isClosing = false;
            grips++;
        }
    }

    private void CloseGripper()
    {
        robotController.SetMotionDirection(leftGripperIndex, RotationDirection.Negative);
        robotController.SetMotionDirection(rightGripperIndex, RotationDirection.Positive);

        // apply indicator color to grippers
        HighlightJoint(leftGripperIndex);
        HighlightJoint(rightGripperIndex);
    }

    private void OpenGripper()
    {
        robotController.SetMotionDirection(leftGripperIndex, RotationDirection.Positive);
        robotController.SetMotionDirection(rightGripperIndex, RotationDirection.Negative);

        // reset color indicator on grippers
        ResetJointColors(leftGripperIndex);
        ResetJointColors(rightGripperIndex);
    }

    private void HighlightJoint(int jointIndex)
    {
        foreach (var renderer in GetRenderers(jointIndex))
        {
            renderer.material.SetColor("_Color", gripperClosedColor);
        }
    }

    private void StoreJointColors(int jointIndex)
    {
        Renderer[] renderers = GetRenderers(jointIndex);
        defaultColors = new Color[renderers.Length];
        for (int index = 0; index < renderers.Length; index++)
        {
            defaultColors[index] = renderers[index].material.color;
        }
    }

    private void ResetJointColors(int jointIndex)
    {
        Renderer[] renderers = GetRenderers(jointIndex);
        for (int index = 0; index < renderers.Length; index++)
        {
            renderers[index].material.SetColor("_Color", defaultColors[index]);
        }
    }

    private Renderer[] GetRenderers(int jointIndex)
    {
        return articulationChain[jointIndex].transform.GetChild(0).GetComponentsInChildren<Renderer>();
    }

    private GameObject SetupGripperCenter()
    {
        GameObject center = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        center.transform.position = articulationChain[gripperBaseIndex].transform.position + new Vector3(0.0f, 0.05f, 0.45f);
        center.transform.rotation = articulationChain[gripperBaseIndex].transform.rotation;
        center.transform.parent = articulationChain[gripperBaseIndex].transform;  // "lock" onto the gripper to mimic translation, rotation, etc.
        center.transform.localScale = new Vector3(0.01f, 0.01f, 0.01f);
        center.GetComponent<Collider>().enabled = false;
        Material mat = center.GetComponent<Renderer>().material;
        mat.SetColor("_Color", new Color(0.4f, 0.4f, 0.4f, 0.3f));
        mat.SetOverrideTag("RenderType", "Transparent");
        mat.SetInt("_SrcBlend", (int)UnityEngine.Rendering.BlendMode.One);
        mat.SetInt("_DstBlend", (int)UnityEngine.Rendering.BlendMode.OneMinusSrcAlpha);
        mat.SetInt("_ZWrite", 0);
        mat.DisableKeyword("_ALPHATEST_ON");
        mat.DisableKeyword("_ALPHABLEND_ON");
        mat.EnableKeyword("_ALPHAPREMULTIPLY_ON");
        mat.renderQueue = 3000;
        center.GetComponent<Renderer>().material = mat;
        return center;
    }
}
