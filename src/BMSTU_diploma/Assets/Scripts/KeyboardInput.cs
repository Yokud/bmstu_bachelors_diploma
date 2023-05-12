using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[RequireComponent(typeof(CharacterController))]
[AddComponentMenu("Control Script/Keyboard Input")]
public class KeyboardInput : MonoBehaviour
{
    public float speed = 60f;

    CharacterController _charController;
    // Start is called before the first frame update
    void Start()
    {
        _charController = GetComponent<CharacterController>();
    }

    // Update is called once per frame
    void Update()
    {
        float deltaX = Input.GetAxis("Horizontal") * speed;
        float deltaZ = Input.GetAxis("Vertical") * speed;
        float deltaY = 0;

        if (Input.GetKey(KeyCode.Space))
        {
            deltaY += speed;
        }
        else if (Input.GetKey(KeyCode.LeftShift))
        {
            deltaY -= speed;
        }

        var movement = new Vector3(deltaX, deltaY, deltaZ);

        movement = Vector3.ClampMagnitude(movement, speed);
        
        movement *= Time.deltaTime;
        transform.Translate(movement, Space.World);
        _charController.Move(movement);
    }
}
