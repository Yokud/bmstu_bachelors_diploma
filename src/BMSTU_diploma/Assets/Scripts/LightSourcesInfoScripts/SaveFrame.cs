using System.Collections;
using System.Collections.Generic;
using System.IO;
using UnityEditor;
using UnityEngine;
using UnityEngine.UI;

public class SaveFrame : MonoBehaviour
{
    public Button SavePanoButton;

    private void Start()
    {
        Button btn = SavePanoButton.GetComponent<Button>();
        btn.onClick.AddListener(ReadPano);
    }

    public void ReadPano()
    {
        string path = EditorUtility.OpenFilePanel("Select frame environment image", "", "png");
        if (path.Length != 0)
        {
            byte[] fileData = File.ReadAllBytes(path);
            var tex = new Texture2D(2, 2, TextureFormat.RGB24, false);
            tex.LoadImage(fileData);
            var fileContent = OpenCvSharp.Unity.TextureToMat(tex);

            EnvDataFields.Frame = fileContent;
        }
    }
}
