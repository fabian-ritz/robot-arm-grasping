using Unity.Robotics;
using UnityEngine;
using System;

public class KeyboardRobotControl : MonoBehaviour
{
    private ArticulationBody[] articulationChain;

    // Stores original colors of the part being highlighted
    private Color[] prevColor;
    private int previousIndex;
    private int selectedIndex;
    private string selectedJoint;
    private RobotController robotController;
    private AutomatedGripper automatedGripper;

    [Tooltip("Color to highlight the currently selected join")]
    public Color highLightColor = new(1.0f, 0.0f, 0.0f, 1.0f);

    [Tooltip("Color to highlight closed grippers")]
    public float gripperCloseThreshold;
    public float gripperOpenThreshold;
    public Color gripperClosedColor = new(0.0f, 1.0f, 0.0f, 1.0f);

    [Tooltip("The object the gripper will react to")]
    public GameObject target;

    // Start is called before the first frame update
    void Start()
    {
        robotController = (RobotController)this.GetComponentInParent(typeof(RobotController));
        if (robotController == null)
        {
            throw new NullReferenceException("Could not find an instance of RobotController");
        }

        previousIndex = selectedIndex = 1;
        articulationChain = this.GetComponentsInChildren<ArticulationBody>();
        DisplaySelectedJoint(selectedIndex);
        StoreJointColors(selectedIndex);

        automatedGripper = new(articulationChain, target, robotController, gripperCloseThreshold,
            gripperOpenThreshold, gripperClosedColor);
    }

    void Update()
    {
        if (Input.GetKeyDown("right"))
        {
            SetSelectedJointIndex(selectedIndex + 1);
            Highlight(selectedIndex);
        }
        else if (Input.GetKeyDown("left"))
        {
            SetSelectedJointIndex(selectedIndex - 1);
            Highlight(selectedIndex);
        }
        MoveJoint(selectedIndex);

        // use to print any information you would like to debug at runtime
        PrintDebugMessage();
    }

    private void FixedUpdate()
    {
        robotController.CalculateNextMotion();  // based on the desired target received from keyboard input via Update()
        automatedGripper.UpdateGripperState();  // depending on target distance, open or close gripper
        // perform one step of the physics simulation (amongst others, the xDrives perform the calculated motion)
        Physics.Simulate(Time.fixedDeltaTime);
    }

    void PrintDebugMessage()
    {
        // Debug.Log(automatedGripper.GetLeftGripperForce().ToString("F2") + " | " + automatedGripper.GetRightGripperForce().ToString("F2"));
    }

    private void SetSelectedJointIndex(int index)
    {
        if (articulationChain.Length > 0)
        {
            selectedIndex = (index + articulationChain.Length) % articulationChain.Length;
        }
    }

    private void MoveJoint(int selectedIndex)
    {
        if (selectedIndex != previousIndex)
        {
            robotController.SetMotionDirection(previousIndex, 0);
            previousIndex = selectedIndex;
        }
        float moveDirection = Input.GetAxis("Vertical");
        if (moveDirection > 0)
        {
            robotController.SetMotionDirection(selectedIndex, RotationDirection.Positive);
        }
        else if (moveDirection < 0)
        {
            robotController.SetMotionDirection(selectedIndex, RotationDirection.Negative);
        }
        else
        {
            robotController.SetMotionDirection(selectedIndex, RotationDirection.None);
        }
    }

    /// <summary>
    /// Highlights the color of the robot by changing the color of the part to a color set by the user in the inspector window
    /// </summary>
    /// <param name="selectedIndex">Index of the link selected in the Articulation Chain</param>
    private void Highlight(int selectedIndex)
    {
        if (selectedIndex == previousIndex || selectedIndex < 0 || selectedIndex >= articulationChain.Length)
        {
            return;
        }

        // reset colors for the previously selected joint
        ResetJointColors(previousIndex);

        // store colors for the current selected joint
        StoreJointColors(selectedIndex);

        DisplaySelectedJoint(selectedIndex);
        Renderer[] rendererList = articulationChain[selectedIndex].transform.GetChild(0).GetComponentsInChildren<Renderer>();

        // set the color of the selected join meshes to the highlight color
        foreach (var mesh in rendererList)
        {
            MaterialExtensions.SetMaterialColor(mesh.material, highLightColor);
        }
    }

    private void DisplaySelectedJoint(int selectedIndex)
    {
        if (selectedIndex < 0 || selectedIndex >= articulationChain.Length)
        {
            return;
        }
        selectedJoint = articulationChain[selectedIndex].name + " (" + selectedIndex + ")";
    }

    /// <summary>
    /// Stores original color of the part being highlighted
    /// </summary>
    /// <param name="index">Index of the part in the Articulation chain</param>
    private void StoreJointColors(int index)
    {
        Renderer[] materialLists = articulationChain[index].transform.GetChild(0).GetComponentsInChildren<Renderer>();
        prevColor = new Color[materialLists.Length];
        for (int counter = 0; counter < materialLists.Length; counter++)
        {
            prevColor[counter] = MaterialExtensions.GetMaterialColor(materialLists[counter]);
        }
    }

    /// <summary>
    /// Resets original color of the part being highlighted
    /// </summary>
    /// <param name="index">Index of the part in the Articulation chain</param>
    private void ResetJointColors(int index)
    {
        Renderer[] previousRendererList = articulationChain[index].transform.GetChild(0).GetComponentsInChildren<Renderer>();
        for (int counter = 0; counter < previousRendererList.Length; counter++)
        {
            MaterialExtensions.SetMaterialColor(previousRendererList[counter].material, prevColor[counter]);
        }
    }

    public void OnGUI()
    {
        GUIStyle centeredStyle = GUI.skin.GetStyle("Label");
        centeredStyle.alignment = TextAnchor.UpperCenter;
        GUI.Label(new Rect(Screen.width / 2 - 200, 10, 400, 20), "Press left/right arrow keys to select a robot joint.", centeredStyle);
        GUI.Label(new Rect(Screen.width / 2 - 200, 30, 400, 20), "Press up/down arrow keys to move " + selectedJoint + ".", centeredStyle);
    }
}
