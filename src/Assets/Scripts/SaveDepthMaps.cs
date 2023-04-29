using OpenCvSharp;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using UnityEditor;
using UnityEngine;
using UnityEngine.UI;

public class SaveDepthMaps : MonoBehaviour
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
        string path = EditorUtility.OpenFolderPanel("Select environment depth maps", "", "");

        if (path.Length == 0)
            return;

        string[] files = Directory.GetFiles(path);
        List<Mat> photos = new List<Mat>();
        foreach (string file in files)
            if (file.EndsWith(".png"))
                photos.Add(Cv2.ImRead(file));

        var stitcher = Stitcher.Create();
        Mat pano = new();
        stitcher.Stitch(photos, pano);

        var env = PanoReceiver.GetComponent<EnvDataFields>();
        env.SpherePano = pano;
    }
}
