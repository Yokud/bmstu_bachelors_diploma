using OpenCvSharp;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using UnityEngine;
using UnityEngine.UI;

public class LightPosCalc : MonoBehaviour
{
    const double MinPixelsCoveragePercent = 0.98;

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
        Cv2.CvtColor(env.SpherePano, grayscaled, ColorConversionCodes.BGRA2GRAY);

        grayscaled.MinMaxLoc(out double minGray, out double maxGray);
        var avgGray = grayscaled.Mean(grayscaled).Val0;

        if (maxGray / avgGray < 1.5)
        {
            Debug.Log("Can't calculate light source positions, because the sphere map has no observable difference between point light and ambient light");
            return;
        }

        // Histogram analysis
        var hist = new Mat();
        Cv2.CalcHist(new[] { grayscaled }, new[] { 0 }, null, hist, 1, new[] { 256 }, new[] { new Rangef(0, 256) });
        var sumPixels = (double)grayscaled.Width * grayscaled.Height;

        var brightnessVal = 255;
        while (hist.RowRange(brightnessVal, 256).Sum().Val0 / sumPixels < 1 - MinPixelsCoveragePercent)
            brightnessVal--;

        var thresholded = new Mat();
        Cv2.Threshold(grayscaled, thresholded, brightnessVal - 1, 255, ThresholdTypes.Tozero); // Затемняем пиксели, яркость которых меньше brightnessVal
        var filtered = new Mat();
        Cv2.MedianBlur(thresholded, filtered, 7);
    }

    void SavePng(Mat image)
    {
        var tex = OpenCvSharp.Unity.MatToTexture(image);
        var bytes = tex.EncodeToPNG();
        File.WriteAllBytes(@"D:\filtered.png", bytes);
    }
}
