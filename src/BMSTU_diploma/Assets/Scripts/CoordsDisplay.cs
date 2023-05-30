using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class CoordsDisplay : MonoBehaviour
{
    public Text CoordsDisplayText;

    Transform cameraTransform;

    // Start is called before the first frame update
    void Start()
    {
        cameraTransform = transform;
    }

    // Update is called once per frame
    void Update()
    {
        if (CoordsDisplayText != null)
        {
            var x = cameraTransform.position.x;
            var y = cameraTransform.position.y;
            var z = cameraTransform.position.z;

            var theta = cameraTransform.localEulerAngles.y;
            var phi = cameraTransform.localEulerAngles.x;

            CoordsDisplayText.text = $"Position(XYZ):\n\tX = {x:0.00}\n\tY = {y:0.00}\n\tZ = {z:0.00}\nRotation(Polar):\n\tAzimuth:{-(theta > 180 ? theta - 360 : theta):0.00}\n\tZenith:{-(phi > 180 ? phi - 360 : phi):0.00}";
        }
    }
}
