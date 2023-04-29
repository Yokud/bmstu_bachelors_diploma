using OpenCvSharp;
using System.Collections;
using System.Collections.Generic;
using UnityEditor;
using UnityEngine;
using UnityEngine.UI;

public class SaveDepthPano : MonoBehaviour
{
    public Button SavePanoButton;
    public GameObject PanoReceiver;

    private void Start()
    {
        Button btn = SavePanoButton.GetComponent<Button>();
        btn.onClick.AddListener(SaveDepthPanoPhoto);
    }

    public void SaveDepthPanoPhoto()
    {
        string path = EditorUtility.OpenFilePanel("Select spherical environment depth image", "", "png");
        if (path.Length != 0)
        {
            var fileContent = Cv2.ImRead(path);

            var env = PanoReceiver.GetComponent<EnvDataFields>();
            env.SphereDepthPano = fileContent;
        }
    }
}
