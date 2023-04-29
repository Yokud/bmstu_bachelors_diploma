using System.Collections;
using System.Collections.Generic;
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
            Debug.Log("Environment data are not loaded yet");
        else
            Debug.Log("Environment data are loaded");
    }
}
