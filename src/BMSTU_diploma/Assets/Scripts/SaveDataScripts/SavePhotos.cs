using OpenCvSharp;
using OpenCvSharp.Detail;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using Unity.VisualScripting;
using UnityEditor;
using UnityEngine;
using UnityEngine.UI;

public class SavePhotos : MonoBehaviour
{
    public Button SavePanoButton;
    public GameObject PanoReceiver;

    private void Start()
    {
        Button btn = SavePanoButton.GetComponent<Button>();
        btn.onClick.AddListener(ReadPhotosStitchPano);
    }

    public void ReadPhotosStitchPano()
    {
        string path = EditorUtility.OpenFolderPanel("Select environment images", "", "png");

        if (path.Length == 0)
            return;

        string[] files = Directory.GetFiles(path);
        List<Mat> photos = new List<Mat>();
        foreach (string file in files)
            if (file.EndsWith(".png"))
            {
                byte[] fileData = File.ReadAllBytes(file);
                var tex = new Texture2D(2, 2);
                tex.LoadImage(fileData);
                var fileContent = OpenCvSharp.Unity.TextureToMat(tex);

                photos.Add(fileContent);
            }
        var stitcher = Stitcher.Create();
        Mat pano = new();
        stitcher.Stitch(photos, pano);

        var env = PanoReceiver.GetComponent<EnvDataFields>();
        env.SpherePano = pano;
    }
}