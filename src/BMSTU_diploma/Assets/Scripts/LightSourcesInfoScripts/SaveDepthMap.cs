using OpenCvSharp;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using UnityEditor;
using UnityEngine;
using UnityEngine.UI;

public class SaveDepthMap : MonoBehaviour
{
    public Button SavePanoButton;

    private void Start()
    {
        Button btn = SavePanoButton.GetComponent<Button>();
        btn.onClick.AddListener(ReadDepthPano);
    }

    public void ReadDepthPano()
    {
        string path = EditorUtility.OpenFilePanel("Select environment depth image", "", "png");
        if (path.Length != 0)
        {
            byte[] fileData = File.ReadAllBytes(path);
            var tex = new Texture2D(2, 2, TextureFormat.RGB48, false);
            tex.LoadImage(fileData);

            Color[] pixels = tex.GetPixels();

            var rgbPixels = new ushort[pixels.Length * 3];
            for (int i = 0; i < pixels.Length; i++)
            {
                rgbPixels[i] = (ushort)(pixels[i].r * 65535);
                rgbPixels[i + 1] = (ushort)(pixels[i].g * 65535);
                rgbPixels[i + 2] = (ushort)(pixels[i].b * 65535);
            }

            var fileContent = new Mat(tex.height, tex.width, MatType.CV_16UC3, rgbPixels.Reverse().ToArray());

            EnvDataFields.DepthMap = fileContent;
        }
    }
}
