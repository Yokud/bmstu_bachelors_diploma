using OpenCvSharp;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using UnityEngine;
using UnityEngine.UI;

public class LightPosCalc : MonoBehaviour
{
    const double MinPixelsCoveragePercent = 0.98;

    public int MedianBlurWindow = 7;
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

        int spherePanoWidth = env.SpherePano.Width, spherePanoHeight = env.SpherePano.Height;
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
        var sumPixels = (double)spherePanoWidth * spherePanoHeight;

        var brightnessVal = 255;
        while (hist.RowRange(brightnessVal, 256).Sum().Val0 / sumPixels < 1 - MinPixelsCoveragePercent)
            brightnessVal--;

        var thresholded = new Mat();
        Cv2.Threshold(grayscaled, thresholded, brightnessVal - 1, 255, ThresholdTypes.Tozero); // ��������� �������, ������� ������� ������ brightnessVal
        var filtered = new Mat();
        Cv2.MedianBlur(thresholded, filtered, MedianBlurWindow);

        Cv2.FindContours(filtered, out Point[][] contours, out HierarchyIndex[] hierarchy, RetrievalModes.Tree, ContourApproximationModes.ApproxSimple);

        //var contoursImage = Mat.Zeros(filtered.Size(), filtered.Type()).ToMat();
        //for (int i = 0; i < contours.Length; i++)
        //{
        //    Cv2.DrawContours(contoursImage, contours, i, Scalar.White, 1, hierarchy: hierarchy, maxLevel: 0);
        //}
        //SavePng(contoursImage, @"D:\testContours.png");

        var moments = new Moments[contours.Length];
        var centroids = new Point[contours.Length];
        var radiuses = new float[contours.Length];
        for (int i = 0; i < contours.Length; i++)
        {
            moments[i] = Cv2.Moments(contours[i]);
            centroids[i] = new Point(moments[i].M10 / moments[i].M00, moments[i].M01 / moments[i].M00);
            radiuses[i] = GetAverageContourDepth(env, contours[i]);
        }

        var polarCoords = new Vector3[centroids.Length];
        var decartCoords = new Vector3[centroids.Length];
        for (int i = 0; i < centroids.Length; i++)
        {
            polarCoords[i] = new Vector3(2 * Mathf.PI * centroids[i].X / spherePanoWidth, Mathf.PI * (spherePanoHeight - centroids[i].Y) / spherePanoHeight, radiuses[i]);
            decartCoords[i] = new Vector3(polarCoords[i].z * Mathf.Sin(polarCoords[i].x) * Mathf.Cos(polarCoords[i].y), polarCoords[i].z * Mathf.Sin(polarCoords[i].x) * Mathf.Sin(polarCoords[i].y), polarCoords[i].z * Mathf.Cos(polarCoords[i].y));
        }

        var ds = LightCoordsReceiver.GetComponent<DataStorageInfo>();
        ds.LightCoords = decartCoords.ToList();
    }

    float GetAverageContourDepth(EnvDataFields env, Point[] contour, int notValidValue = 0)
    {
        var minX = contour.Min(p => p.X);
        var maxX = contour.Max(p => p.X);
        var minY = contour.Min(p => p.Y);
        var maxY = contour.Max(p => p.Y);

        float sumDepth = 0;
        var validPoints = 0;

        for (int i = minX; i < maxX; i++) 
            for (int j = minY; j < maxY; j++)
                if (env.SphereDepthPano.At<int>(i, j) != notValidValue)
                {
                    sumDepth += (env.SphereDepthPano.At<int>(i, j) - 1) / 254f * 3.2f + 0.8f;
                    validPoints++;
                }

        return sumDepth / validPoints;
    }

    void SavePng(Mat image, string filename)
    {
        var tex = OpenCvSharp.Unity.MatToTexture(image);
        var bytes = tex.EncodeToPNG();
        File.WriteAllBytes(filename, bytes);
    }
}
