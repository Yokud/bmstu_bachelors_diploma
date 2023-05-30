using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CameraRotate : MonoBehaviour
{
    public float sensivityHor = 4.5f;
    public float sensivityVer = 4.5f;

    public float minVert = -90f;
    public float maxVert = 90f;

    public GameObject SceneManager;

    float _rotationX = 0;
    ExamplesManager _examplesManager;

    void Start()
    {
        _examplesManager = SceneManager.GetComponent<ExamplesManager>();
    }

    // Update is called once per frame
    void Update()
    {
        float deltaY = 0, deltaX = 0;

        if (Input.GetKey(KeyCode.T))
            deltaX -= sensivityHor;
        else if (Input.GetKey(KeyCode.G))
            deltaX += sensivityHor;
        else if (Input.GetKey(KeyCode.R))
            deltaY += sensivityVer;
        else if (Input.GetKey(KeyCode.F))
            deltaY -= sensivityVer;

        _rotationX -= deltaY;
        _rotationX = Mathf.Clamp(_rotationX, minVert, maxVert);

        float rotationY = transform.localEulerAngles.y + deltaX;

        if (!_examplesManager.HasSelectedExample)
            transform.localEulerAngles = new Vector3(_rotationX, rotationY, 0);
        else
            _rotationX = 0;
    }
}
