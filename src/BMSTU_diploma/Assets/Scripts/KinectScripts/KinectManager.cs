using OpenCvSharp;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Runtime.CompilerServices;
using System.Threading.Tasks;
using UnityEngine;
using UnityEngine.UI;

public class KinectManager : MonoBehaviour
{
    public GameObject PlaneGrid;
    public GameObject Background;

    public int MinMatchPointsCount = 4;

    // Bool to keep track of whether Kinect has been initialized
    private bool kinectInitialized = false;

    //Matrix4x4 kinectToWorld, flipMatrix;
    static KinectManager instance;

    Camera cam;

    // Image stream handles for the kinect
    private IntPtr colorStreamHandle;
    private IntPtr depthStreamHandle;

    private int mapSize;

    // Depth data
    private ushort[] depthMap;

    // Color data
    private Texture2D ColorTexture;
    private Color32[] colorImage;
    private byte[] colorMap;

    // Mesh info
    Vector3[] newVertices;
    Vector3[] newNormals;
    int[] newTriangles;
    Mesh myMesh;
    float[] NormalizedDepthValues;
    const float verticesDepthDeltaTreshold = 0.15f;

    readonly int Width = KinectWrapper.GetDepthWidth();
    readonly int Height = KinectWrapper.GetDepthHeight();
    readonly int MinDepthValue = KinectWrapper.GetMinDepth();
    readonly int MaxDepthValue = KinectWrapper.GetMaxDepth();
    public float MeshHeight;

    Image bg;

    Mat frame;

    AKAZE akaze = AKAZE.Create();
    Mat panoDescr = new();
    KeyPoint[] panoKeyPoints;

    public Text CalibrationText;

    // returns the single KinectManager instance
    public static KinectManager Instance => instance;

    public static bool IsKinectInitialized() => instance != null && instance.kinectInitialized;

    void Awake()
    {
        int hr = 0;

        try
        {
            hr = KinectWrapper.NuiInitialize(KinectWrapper.NuiInitializeFlags.UsesDepth | KinectWrapper.NuiInitializeFlags.UsesColor);
            if (hr != 0)
            {
                throw new Exception("NuiInitialize Failed");
            }

            depthStreamHandle = IntPtr.Zero;
            hr = KinectWrapper.NuiImageStreamOpen(KinectWrapper.NuiImageType.Depth,
                KinectWrapper.Constants.DepthImageResolution, 0, 2, IntPtr.Zero, ref depthStreamHandle);
            if (hr != 0)
            {
                throw new Exception("Cannot open depth stream");
            }

            colorStreamHandle = IntPtr.Zero;
            hr = KinectWrapper.NuiImageStreamOpen(KinectWrapper.NuiImageType.Color,
                KinectWrapper.Constants.ColorImageResolution, 0, 2, IntPtr.Zero, ref colorStreamHandle);
            if (hr != 0)
            {
                throw new Exception("Cannot open color stream");
            }

            instance = this;
            DontDestroyOnLoad(gameObject);
        }
        catch (DllNotFoundException e)
        {
            string message = "Please check the Kinect SDK installation.";
            Debug.LogError(message);
            Debug.LogError(e.ToString());
            if (CalibrationText != null)
            {
                CalibrationText.color = Color.red;
                CalibrationText.text = "Kinect status: " + message;
            }
            return;
        }
        catch (Exception e)
        {
            string message = "Kinect status: " + e.Message + " - " + KinectWrapper.GetNuiErrorString(hr);
            Debug.LogError(message);
            Debug.LogError(e.ToString());
            if (CalibrationText != null)
            {
                CalibrationText.color = Color.red;
                CalibrationText.text = message;
            }

            return;
        }

        // Initialize depth map related stuff
        mapSize = KinectWrapper.GetDepthWidth() * KinectWrapper.GetDepthHeight();
        depthMap = new ushort[mapSize];

        // Initialize color map related stuff
        ColorTexture = new Texture2D(KinectWrapper.GetColorWidth(), KinectWrapper.GetColorHeight());
        colorImage = new Color32[KinectWrapper.GetColorWidth() * KinectWrapper.GetColorHeight()];
        colorMap = new byte[colorImage.Length << 2];

        // GUI Text.
        if (CalibrationText != null)
        {
            CalibrationText.color = Color.black;
            CalibrationText.text = "Kinect status: Kinect initialized";
        }

        Debug.Log("Kinect initialized.");

        kinectInitialized = true;
    }

    // Make sure to kill the Kinect on quitting.
    void OnApplicationQuit()
    {
        KinectShutdown();
    }

    public static void KinectShutdown()
    {
        if (IsKinectInitialized())
        {
            KinectWrapper.NuiShutdown();
            instance.kinectInitialized = false;
            instance = null;
        }
    }

    // Start is called before the first frame update
    void Start()
    {
        myMesh = new Mesh
        {
            indexFormat = UnityEngine.Rendering.IndexFormat.UInt32
        };
        PlaneGrid.GetComponent<MeshFilter>().mesh = myMesh;
        PlaneGrid.GetComponent<MeshCollider>().sharedMesh = myMesh;

        bg = Background.GetComponentInChildren<Image>();

        cam = Camera.main;

        GetFrustumParams(Background.transform.position.z, out float bgHeight, out float bgWidth);
        var rt = Background.GetComponent<RectTransform>();
        rt.sizeDelta = new Vector2(bgWidth, bgHeight);

        akaze.DetectAndCompute(EnvDataFields.SpherePano, new Mat(), out panoKeyPoints, panoDescr);

        SetupArrays();
    }

    // Update is called once per frame
    void Update()
    {
        if (kinectInitialized)
        {
            if (depthStreamHandle != IntPtr.Zero && KinectWrapper.PollDepth(depthStreamHandle, KinectWrapper.Constants.IsNearMode, ref depthMap))
            {
                NormalizeDepthValues();
                UpdateMesh();
            }

            if (colorStreamHandle != IntPtr.Zero && KinectWrapper.PollColor(colorStreamHandle, ref colorMap, ref colorImage))
            {
                UpdateColorMap();
                if (bg != null)
                    bg.sprite = Sprite.Create(ColorTexture, new UnityEngine.Rect(0 ,0, Width, Height), new Vector2());
                UpdateCameraOrientation();
            }
        }
    }

    void SetupArrays()
    {
        NormalizedDepthValues = new float[Width * Height];
        newVertices = new Vector3[Width * Height];
        newNormals = new Vector3[Width * Height];
        newTriangles = new int[(Width - 1) * (Height - 1) * 6];

        for (int H = 0; H < Height; H++)
        {
            for (int W = 0; W < Width; W++)
            {
                int Index = GetArrayIndex(W, H);
                newVertices[Index] = cam.ScreenToWorldPoint(new Vector3(W, H, PlaneGrid.transform.position.z));
                newVertices[Index].z -= PlaneGrid.transform.position.z;
                newNormals[Index] = new Vector3(0, 0, -1);

                if ((W != (Width - 1)) && (H != (Height - 1)))
                {
                    int TopLeft = Index;
                    int TopRight = Index + 1;
                    int BotLeft = Index + Width;
                    int BotRight = Index + 1 + Width;

                    int TrinagleIndex = (W + H * (Width - 1)) * 6;
                    newTriangles[TrinagleIndex] = TopLeft;
                    newTriangles[TrinagleIndex + 1] = BotLeft;
                    newTriangles[TrinagleIndex + 2] = TopRight;
                    newTriangles[TrinagleIndex + 3] = BotLeft;
                    newTriangles[TrinagleIndex + 4] = BotRight;
                    newTriangles[TrinagleIndex + 5] = TopRight;
                }
            }
        }

        myMesh.Clear();
        myMesh.vertices = newVertices;
        myMesh.normals = newNormals;
        myMesh.triangles = newTriangles;
    }

    private void GetFrustumParams(float distance, out float frustumHeight, out float frustumWidth)
    {
        frustumHeight = 2.0f * distance * Mathf.Tan(cam.fieldOfView * 0.5f * Mathf.Deg2Rad);
        frustumWidth = frustumHeight * cam.aspect;
    }

    void NormalizeDepthValues()
    {
        Parallel.For(0, mapSize, (i) =>
        {
            int ImageValue = KinectWrapper.GetRealDepth(depthMap[mapSize - i - 1]);

            //Clamp Value
            ImageValue = Mathf.Clamp(ImageValue, MinDepthValue, MaxDepthValue);

            //Calculate
            NormalizedDepthValues[i] = (ImageValue - MinDepthValue) / (float)(MaxDepthValue - MinDepthValue);
        });
    }

    void UpdateMesh()
    {
        var planeGridZ = PlaneGrid.transform.position.z;
        var conversionCameraToWorldMatrix = cam.cameraToWorldMatrix * cam.projectionMatrix.inverse;
        Parallel.For(0, mapSize, (i) =>
        {
            var screenCoords = GetScreenCoords(i);
            newVertices[i] = ManualScreenToWorldPoint(screenCoords, NormalizedDepthValues[i] * MeshHeight + planeGridZ, conversionCameraToWorldMatrix);
            newVertices[i].z -= planeGridZ;
        });

        Parallel.For(0, Height - 1, (H) =>
        {
            for (int W = 0; W < Width - 1; W++)
            {
                int Index00 = GetArrayIndex(W, H);
                int Index10 = GetArrayIndex(W + 1, H);
                int Index01 = GetArrayIndex(W, H + 1);
                int Index11 = GetArrayIndex(W + 1, H + 1);
                float avgDepth = (NormalizedDepthValues[Index00] + NormalizedDepthValues[Index10] + NormalizedDepthValues[Index01] + NormalizedDepthValues[Index11]) / 4f;

                if (Mathf.Abs(NormalizedDepthValues[Index00] - avgDepth) < verticesDepthDeltaTreshold &&
                    Mathf.Abs(NormalizedDepthValues[Index10] - avgDepth) < verticesDepthDeltaTreshold
                    && Mathf.Abs(NormalizedDepthValues[Index01] - avgDepth) < verticesDepthDeltaTreshold
                    && Mathf.Abs(NormalizedDepthValues[Index11] - avgDepth) < verticesDepthDeltaTreshold)
                {
                    int TopLeft = Index00;
                    int TopRight = Index00 + 1;
                    int BotLeft = Index00 + Width;
                    int BotRight = Index00 + 1 + Width;

                    int TrinagleIndex = (W + H * (Width - 1)) * 6;
                    newTriangles[TrinagleIndex] = TopLeft;
                    newTriangles[TrinagleIndex + 1] = BotLeft;
                    newTriangles[TrinagleIndex + 2] = TopRight;
                    newTriangles[TrinagleIndex + 3] = BotLeft;
                    newTriangles[TrinagleIndex + 4] = BotRight;
                    newTriangles[TrinagleIndex + 5] = TopRight;
                }
                else
                {
                    int TrinagleIndex = (W + H * (Width - 1)) * 6;
                    newTriangles[TrinagleIndex] = 0;
                    newTriangles[TrinagleIndex + 1] = 0;
                    newTriangles[TrinagleIndex + 2] = 0;
                    newTriangles[TrinagleIndex + 3] = 0;
                    newTriangles[TrinagleIndex + 4] = 0;
                    newTriangles[TrinagleIndex + 5] = 0;
                }
            }
        });

        myMesh.vertices = newVertices;
        myMesh.triangles = newTriangles;
        myMesh.RecalculateNormals();
    }

    /// <summary>
    /// Manual version of camera.ScreenToWorldPoint
    /// </summary>
    /// <param name="screenPoint">X and Y coords</param>
    /// <param name="distance">Z coord</param>
    /// <param name="cameraToWorldMatrixProjectionMatrixInverse">camera.cameraToWorldMatrix * camera.projectionMatrix.inverse</param>
    /// <returns></returns>
    public Vector3 ManualScreenToWorldPoint(Vector2 screenPoint, float distance, Matrix4x4 cameraToWorldMatrixProjectionMatrixInverse)
    {
        // here we are converting screen point in screen space to camera space point placed on a plane "distance" away from the camera
        // screen point is in range [(0,0) - (Screen.Width, Screen.Height)]
        Vector2 pointViewportSpace = screenPoint / new Vector2(Width, Height); // convert space [(0,0) - (Screen.Width, Screen.Height)] to [(0,0) - (1,1)]
        Vector2 pointCameraSpaceNormalized = (pointViewportSpace * 2.0f) - Vector2.one; // convert space [(0,0) - (1,1)] to [(-1,-1) - (1,1)]
        Vector2 pointCameraSpace = pointCameraSpaceNormalized * distance; // convert space [(-1,-1) - (1,1)] to [(-dist,-dist) - (dist, dist)]
        Vector4 planePoint = new(pointCameraSpace.x, pointCameraSpace.y, distance, distance); // define the point (don't know why z and w components need to be set to distance)

        // calculate convertion matrix from camera space to world space
        //Matrix4x4 matrix = cameraToWorldMatrix * projectionMatrixInverse;
        // multiply world point by VP matrix
        Vector4 worldPoint = cameraToWorldMatrixProjectionMatrixInverse * planePoint;

        return worldPoint;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    int GetArrayIndex(int W, int H) => W + H * Width;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    Vector2 GetScreenCoords(int index) => new(index % Width, index / Width);

    // Update the Color Map
    void UpdateColorMap()
    {
        ColorTexture.SetPixels32(colorImage);
        ColorTexture.Apply();
        frame = OpenCvSharp.Unity.TextureToMat(ColorTexture);
    }

    void UpdateCameraOrientation()
    {
        if (EnvDataFields.SpherePano == null || EnvDataFields.SphereDepthPano == null)
            return;

        Mat frameDescr = new();

        akaze.DetectAndCompute(frame, new Mat(), out KeyPoint[] frameKeyPoints, frameDescr);

        DescriptorMatcher matcher = DescriptorMatcher.Create("BruteForce-Hamming");
        var knnMatches = matcher.KnnMatch(frameDescr, panoDescr, 2);

        float ratioThreshold = 0.8f; // Nearest neighbor matching ratio
        List<Point2f> listOfMatchedPano = new();
        List<Point2f> listOfMatchedFrame = new();
        for (int i = 0; i < knnMatches.Length; i++)
        {
            DMatch[] matches = knnMatches[i];
            float dist1 = matches[0].Distance;
            float dist2 = matches[1].Distance;
            if (dist1 < ratioThreshold * dist2)
            {
                listOfMatchedPano.Add(panoKeyPoints[matches[0].TrainIdx].Pt);
                listOfMatchedFrame.Add(frameKeyPoints[matches[0].QueryIdx].Pt);
            }
        }

        if (listOfMatchedPano.Count < MinMatchPointsCount)
        {
            Debug.Log("Too less match points");
            return;
        }

        var matchedPoints = listOfMatchedPano.Count;
        Point3f[] pano3D = new Point3f[matchedPoints];
        Point3f[] frame3D = new Point3f[matchedPoints];
        var conversionCameraToWorldMatrix = cam.cameraToWorldMatrix * cam.projectionMatrix.inverse;
        for (int i = 0; i < matchedPoints; i++)
        {
            pano3D[i] = GetDecartCoordsFromPanos(listOfMatchedPano[i]);

            var worldFramePoint = ManualScreenToWorldPoint(new Vector2(listOfMatchedFrame[i].X, listOfMatchedFrame[i].Y), KinectWrapper.GetRealDepth(depthMap[mapSize - GetArrayIndex(Mathf.RoundToInt(listOfMatchedFrame[i].X), Mathf.RoundToInt(listOfMatchedFrame[i].Y)) - 1]) / 10f, conversionCameraToWorldMatrix);
            frame3D[i] = new Point3f(worldFramePoint.x, worldFramePoint.y, worldFramePoint.z);
        }

        Point3f meanPano = new(), meanFrame = new();
        for (int i = 0; i < matchedPoints; i++)
        {
            meanPano += pano3D[i];
            meanFrame += frame3D[i];
        }

        meanPano = new Point3f(meanPano.X / matchedPoints, meanPano.Y / matchedPoints, meanPano.Z / matchedPoints);
        meanFrame = new Point3f(meanFrame.X / matchedPoints, meanFrame.Y / matchedPoints, meanFrame.Z / matchedPoints);

        Point3f[] pano3DDecentrized = new Point3f[matchedPoints];
        Point3f[] frame3DDecentrized = new Point3f[matchedPoints];

        for (int i = 0; i < matchedPoints; i++)
        {
            pano3DDecentrized[i] = pano3D[i] - meanPano;
            frame3DDecentrized[i] = frame3D[i] - meanFrame;
        }

        float[] pano3DDecentrValues = new float[matchedPoints * 3];
        float[] frame3DDecentrValues = new float[matchedPoints * 3];

        for (int i = 0; i < matchedPoints * 3; i += 3) 
        {
            pano3DDecentrValues[i] = pano3DDecentrized[i / 3].X;
            pano3DDecentrValues[i + 1] = pano3DDecentrized[i / 3].Y;
            pano3DDecentrValues[i + 2] = pano3DDecentrized[i / 3].Z;

            frame3DDecentrValues[i] = frame3DDecentrized[i / 3].X;
            frame3DDecentrValues[i + 1] = frame3DDecentrized[i / 3].Y;
            frame3DDecentrValues[i + 2] = frame3DDecentrized[i / 3].Z;
        }

        Mat pano3dDecentrMat = new(pano3DDecentrized.Length, 3, MatType.CV_32F, pano3DDecentrValues);
        Mat frame3dDecentrMat = new(frame3DDecentrized.Length, 3, MatType.CV_32F, frame3DDecentrValues);

        Mat h = frame3dDecentrMat.Transpose() * pano3dDecentrMat;

        Mat u = new(), w = new(), vt = new();

        SVD.Compute(h, w, u, vt);

        Mat r = vt.Transpose() * u.Transpose();

        if (r.Determinant() < 0)
        {
            Debug.Log("Reflection detected");
            Mat uLocal = new(), sLocal = new(), vtLocal = new();
            SVD.Compute(r, sLocal, uLocal, vtLocal);

            vtLocal.Set(2, 0, -vtLocal.At<float>(2, 0));
            vtLocal.Set(2, 1, -vtLocal.At<float>(2, 1));
            vtLocal.Set(2, 2, -vtLocal.At<float>(2, 2));

            r = vtLocal.Transpose() * uLocal.Transpose();
        }

        Mat t = -r * new Mat(3, 1, MatType.CV_32F, new[] { meanFrame.X, meanFrame.Y, meanFrame.Z }) + new Mat(3, 1, MatType.CV_32F, new[] { meanPano.X, meanPano.Y, meanPano.Z });

        //Mat rvec = new();
        //Cv2.Rodrigues(r, rvec);

        var pos = new Vector3(t.At<float>(0), t.At<float>(1), t.At<float>(2));
        var rot = Quaternion.LookRotation(new Vector3(r.Get<float>(2, 0), r.Get<float>(2, 1), r.Get<float>(2, 2)), new Vector3(r.Get<float>(1, 0), r.Get<float>(1, 1), r.Get<float>(1, 2)));
        cam.transform.SetPositionAndRotation(pos, rot);
    }

    static Point3f GetDecartCoordsFromPanos(Point2f v)
    {
        var radius = Mathf.Max(EnvDataFields.SphereDepthPano.Get<Vec3w>(Mathf.RoundToInt(v.Y), Mathf.RoundToInt(v.X))[0] / 10f, 80f);
        var polarCoords = new Vector3(radius, 2 * Mathf.PI * v.X / EnvDataFields.SpherePano.Width, Mathf.PI * (EnvDataFields.SpherePano.Height - v.Y) / EnvDataFields.SpherePano.Height);
        return new Point3f(polarCoords.x * Mathf.Sin(polarCoords.y) * Mathf.Cos(polarCoords.z), polarCoords.x * Mathf.Sin(polarCoords.y) * Mathf.Sin(polarCoords.z), polarCoords.x * Mathf.Cos(polarCoords.z));
    }
}
