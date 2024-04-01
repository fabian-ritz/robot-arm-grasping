using System.Collections.Generic;
using UnityEngine;
using System.Linq;

public class GraspingPointCalculator : MonoBehaviour
{
    public List<GameObject> graspingPoints;
    private Vector3 graspingPointSize = new Vector3(.008f, .008f, .008f);
    private Color graspingPointColor = Color.red;

    void Start()
    {
        Vector3 distanceFromCenter = Vector3.Scale(this.GetComponent<Renderer>().bounds.size, new Vector3(0.5f, 0.5f, 0.5f));
        Mesh mesh = GetComponent<MeshFilter>().sharedMesh;
        Vector3[] uniqueNormals = mesh.normals.ToList().Distinct().ToArray();
        foreach (Vector3 normal in uniqueNormals)
        {
            GameObject sphere = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            sphere.transform.position = transform.position + Vector3.Scale(normal, distanceFromCenter);
            sphere.transform.rotation = transform.rotation;
            sphere.transform.parent = transform;
            sphere.transform.localScale = graspingPointSize;
            sphere.GetComponent<Collider>().enabled = false;
            Renderer renderer = sphere.GetComponent<Renderer>();
            renderer.material.SetColor("_Color", graspingPointColor);
            graspingPoints.Add(sphere);
        }
    }
}