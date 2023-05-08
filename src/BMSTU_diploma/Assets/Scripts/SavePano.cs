using OpenCvSharp;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using UnityEditor;
using UnityEngine;
using UnityEngine.UI;

public class SavePano : MonoBehaviour
{
    public Button SavePanoButton;
    public GameObject PanoReceiver;

    private void Start()
    {
        Button btn = SavePanoButton.GetComponent<Button>();
        btn.onClick.AddListener(SavePanoPhoto);
    }

    public void SavePanoPhoto()
    {
        string path = EditorUtility.OpenFilePanel("Select spherical environment image", "", "png");
        if (path.Length != 0)
        {
            byte[] fileData = File.ReadAllBytes(path);
            var tex = new Texture2D(2, 2);
            tex.LoadImage(fileData);
            var fileContent = OpenCvSharp.Unity.TextureToMat(tex);

            var env = PanoReceiver.GetComponent<EnvDataFields>();
            env.SpherePano = fileContent;
        }
    }
}
