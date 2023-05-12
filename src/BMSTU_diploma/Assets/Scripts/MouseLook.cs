using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class MouseLook : MonoBehaviour
{
    public enum RotationAxes
    {
        XandY,
        X,
        Y,
    };

    public RotationAxes axes = RotationAxes.XandY;
    public float sensivityHor = 4.5f;
    public float sensivityVer = 4.5f;

    public float minVert = -90f;
    public float maxVert = 90f;

    float _rotationX = 0;

    // Update is called once per frame
    void Update()
    {
        switch (axes)
        {
            case RotationAxes.X:
                transform.Rotate(0, Input.GetAxis("Mouse X") * sensivityHor, 0);
                break;

            case RotationAxes.Y:
                _rotationX -= Input.GetAxis("Mouse Y") * sensivityVer;
                _rotationX = Mathf.Clamp(_rotationX, minVert, maxVert);

                float rotationY = transform.localEulerAngles.y;

                transform.localEulerAngles = new Vector3(_rotationX, rotationY, 0);
                break;

            default:
                _rotationX -= Input.GetAxis("Mouse Y") * sensivityVer;
                _rotationX = Mathf.Clamp(_rotationX, minVert, maxVert);

                rotationY = transform.localEulerAngles.y + Input.GetAxis("Mouse X") * sensivityHor;

                transform.localEulerAngles = new Vector3(_rotationX, rotationY, 0);
                break;
        }
    }
}
