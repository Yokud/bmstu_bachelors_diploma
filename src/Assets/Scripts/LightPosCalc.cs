using OpenCvSharp;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using UnityEngine;
using UnityEngine.UI;

public class LightPosCalc : MonoBehaviour
{
    public Button StartButton;
    public GameObject EnvironmentData;
    public GameObject LightCoordsReceiver;

    private void Start()
    {
        Button btn = StartButton.GetComponent<Button>();
        btn.onClick.AddListener(Calc);
    }

    public void Calc()
    {
        var env = EnvironmentData.GetComponent<EnvDataFields>();
        if (env.SpherePano is null || env.SphereDepthPano is null)
        {
            Debug.Log("Environment data are not loaded yet");
            return;
        }

        var grayscaled = new Mat();
        Cv2.CvtColor(env.SpherePano, grayscaled, ColorConversionCodes.RGBA2GRAY);

        //var tex = OpenCvSharp.Unity.MatToTexture(grayscaled);
        //var bytes = tex.EncodeToPNG();
        //File.WriteAllBytes(@"D:\gray.png", bytes);


    }
}
