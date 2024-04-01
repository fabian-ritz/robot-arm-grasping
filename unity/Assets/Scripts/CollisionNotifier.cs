using UnityEngine;

public class CollisionNotifier : MonoBehaviour
{
    public GameObject[] colliderParents;

    private void OnTriggerStay(Collider other)
    {
        foreach (GameObject parent in colliderParents)
        {
            if (other.gameObject.Equals(parent))
            {
                other.transform.root.SendMessage("HandleCollision", this.gameObject.name);
            }
        }
    }
}
