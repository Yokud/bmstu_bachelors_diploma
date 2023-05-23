using UnityEngine;

public class ExampleManager : MonoBehaviour
{
    public float MoveSpeed = 20f;
    public float RotateSpeed = 30f;

    Rigidbody rb;

    public void MoveSelectedExample()
    {
        float deltaX = Input.GetAxis("Horizontal") * MoveSpeed;
        float deltaZ = Input.GetAxis("Vertical") * MoveSpeed;
        float deltaY = 0;

        if (Input.GetKey(KeyCode.Space))
            deltaY += MoveSpeed;
        else if (Input.GetKey(KeyCode.LeftShift))
            deltaY -= MoveSpeed;

        var movement = new Vector3(deltaX, deltaY, deltaZ);

        movement = Vector3.ClampMagnitude(movement, MoveSpeed);

        movement *= Time.deltaTime;

        transform.Translate(movement, Space.World);
    }

    public void RotateSelectedExample()
    {
        if (Input.GetKey(KeyCode.R))
            transform.Rotate(RotateSpeed * Time.deltaTime * Vector3.up);
        else if (Input.GetKey(KeyCode.F))
            transform.Rotate(RotateSpeed * Time.deltaTime * Vector3.down);
        else if (Input.GetKey(KeyCode.Y))
            transform.Rotate(RotateSpeed * Time.deltaTime * Vector3.right);
        else if (Input.GetKey(KeyCode.H))
            transform.Rotate(RotateSpeed * Time.deltaTime * Vector3.left);
        else if (Input.GetKey(KeyCode.T))
            transform.Rotate(RotateSpeed * Time.deltaTime * Vector3.forward);
        else if (Input.GetKey(KeyCode.G))
            transform.Rotate(RotateSpeed * Time.deltaTime * Vector3.back);
    }

    private void Start()
    {
        rb = GetComponent<Rigidbody>();
    }
}
