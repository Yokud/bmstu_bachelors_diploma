using OpenCvSharp;
using OpenCvSharp.XFeatures2D;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Threading.Tasks;
using UnityEngine;
using UnityEngine.UI;

public class KinectManager : MonoBehaviour
{
    public GameObject PlaneGrid;
    public GameObject Background;

    public int MinMatchPointsCount = 4;
    public int MaxMatchPointsCount = 10;

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

    readonly SIFT keyPointsAlg = SIFT.Create();
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

        keyPointsAlg.DetectAndCompute(EnvDataFields.SpherePano, new Mat(), out panoKeyPoints, panoDescr);

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
                UpdateBackground();
                UpdateCameraOrientation();
            }
        }
    }

    private void UpdateBackground()
    {
        var bgPos = cam.transform.TransformPoint(0, 0, 400);
        Background.transform.SetPositionAndRotation(bgPos, Quaternion.LookRotation(cam.transform.forward, cam.transform.up));

        if (bg != null)
            bg.sprite = Sprite.Create(ColorTexture, new UnityEngine.Rect(0, 0, Width, Height), new Vector2());
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
                newVertices[Index] = cam.ScreenToWorldPoint(new Vector3(W, H, 80));
                newVertices[Index].z -= 80;
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

        var pgPos = cam.transform.TransformPoint(0, 0, 80);
        PlaneGrid.transform.SetPositionAndRotation(pgPos, Quaternion.LookRotation(cam.transform.forward, cam.transform.up));

        var camZ = cam.transform.position.z;
        var planeGridZ = 80;
        var conversionCameraToPlaneGridMatrix = PlaneGrid.transform.worldToLocalMatrix * cam.cameraToWorldMatrix * cam.projectionMatrix.inverse;
        Parallel.For(0, mapSize, (i) =>
        {
            var screenCoords = GetScreenCoords(i);
            newVertices[i] = ManualScreenToWorldPoint(screenCoords, NormalizedDepthValues[i] * MeshHeight + planeGridZ, conversionCameraToPlaneGridMatrix);
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

        keyPointsAlg.DetectAndCompute(frame, new Mat(), out KeyPoint[] frameKeyPoints, frameDescr);

        //KDTreeIndexParams indexParams = new KDTreeIndexParams(5);
        //SearchParams searchParams = new SearchParams(50);
        //var matcher = new FlannBasedMatcher(indexParams, searchParams);
        var matcher = new BFMatcher(NormTypes.L2, true);
        //var knnMatches = matcher.KnnMatch(frameDescr, panoDescr, 2);
        
        var matches = matcher.Match(frameDescr, panoDescr);

        var goodMatches = matches.OrderBy(x => x.Distance).Take(MaxMatchPointsCount).ToArray();



        //List<Point2f> listOfMatchedPano = new();
        //List<Point2f> listOfMatchedFrame = new();

        //List<DMatch> goodMatches = new();
        //List<KeyPoint> matchesPano = new();
        //List<KeyPoint> matchesFrame = new();
        //var nn_match_ratio = 0.7;
        //for (int i = 0; i < matches.Length; i++)
        //{
        //    var match = matches[i];
        //    if (match[0].Distance < nn_match_ratio * match[1].Distance)
        //    {
        //        goodMatches.Add(match[0]);
        //        matchesPano.Add(panoKeyPoints[match[0].TrainIdx]);
        //        matchesFrame.Add(frameKeyPoints[match[0].QueryIdx]);
        //    }
        //    //listOfMatchedFrame.Add(frameKeyPoints[goodMatches[i].QueryIdx].Pt);
        //    //listOfMatchedPano.Add(panoKeyPoints[goodMatches[i].TrainIdx].Pt);
        //}

        //List<DMatch> goodMatches = new();
        //var inlier_threshold = 2.5;
        //for (int i = 0; i < matchesFoundPano.Count; i++)
        //{

        //}

        //var goodMatchesArr = goodMatches.OrderBy(x => x.Distance).Take(MaxMatchPointsCount).ToArray();
        //var matchesPano = goodMatchesArr.Select(m => panoKeyPoints[m.TrainIdx]).ToArray();
        //var matchesFrame = goodMatchesArr.Select(m => frameKeyPoints[m.QueryIdx]).ToArray();

        //Mat testImg = new();
        //Cv2.DrawMatches(frame, frameKeyPoints, EnvDataFields.SpherePano, panoKeyPoints, goodMatches, testImg);
        //LightPosCalc.SavePng(testImg, "D:\\test_kp.png");

        if (goodMatches.Length < MinMatchPointsCount)
        {
            Debug.Log("Too less match points");
            return;
        }

        var matchesPano = goodMatches.Select(m => panoKeyPoints[m.TrainIdx]).ToArray();
        var matchesFrame = goodMatches.Select(m => frameKeyPoints[m.QueryIdx]).ToArray();

        var matchedPoints = goodMatches.Length;
        List<Point3f> pano3D = new(matchedPoints);
        List<Point2f> sortedMatñhedFrame = new(matchedPoints);
        for (int i = 0; i < matchedPoints; i++)
        {
            //pano3D[i] = GetDecartCoordsFromPanos(listOfMatchedPano[i]);
            var pano3DPoint = GetDecartCoordsFromPanos(matchesPano[i].Pt, 4);
            if (!(pano3DPoint.X == 0 && pano3DPoint.Y == 0 && pano3DPoint.Z == 0))
            {
                pano3D.Add(pano3DPoint);
                sortedMatñhedFrame.Add(matchesFrame[i].Pt);
            }
        }

        if (pano3D.Count < MinMatchPointsCount)
        {
            Debug.Log("Too less 3D match points");
            return;
        }

        matchedPoints = pano3D.Count;

        var cameraMatrix = new double[3, 3];
        cameraMatrix[0, 0] = Width;
        cameraMatrix[1, 1] = Width;
        cameraMatrix[0, 2] = Width / 2;
        cameraMatrix[1, 2] = Height / 2;
        cameraMatrix[2, 2] = 1;

        var distCoefs = Enumerable.Repeat<double>(0, 5);

        Cv2.SolvePnPRansac(pano3D, sortedMatñhedFrame, cameraMatrix, distCoefs, out double[] rvec, out double[] tvec);

        var t = new Vector3((float)tvec[0], (float)tvec[1], (float)tvec[2]);

        Cv2.Rodrigues(rvec, out double[,] r);

        Matrix4x4 R = new();
        R.SetRow(0, new((float)r[0, 0], (float)r[0, 1], (float)r[0, 2]));
        R.SetRow(1, new((float)r[1, 0], (float)r[1, 1], (float)r[1, 2]));
        R.SetRow(2, new((float)r[2, 0], (float)r[2, 1], (float)r[2, 2]));
        R.SetRow(3, new(0, 0, 0, 1));

        R = R.transpose;

        Matrix4x4 negativeMatrix = new();
        negativeMatrix.SetRow(0, new Vector4(-1, 0, 0));
        negativeMatrix.SetRow(1, new Vector4(0, -1, 0));
        negativeMatrix.SetRow(2, new Vector4(0, 0, -1));
        negativeMatrix.SetRow(3, new Vector4(0, 0, 0, 1));
        var pos = negativeMatrix * R * t;

        //Quaternion Q = R.rotation;
        //cam.transform.SetPositionAndRotation(Vector3.zero, new Quaternion(-Q.x, Q.y, -Q.z, Q.w));

        var y_axis = new Vector3(R[0, 1], R[1, 1], R[2, 1]);
        var z_axis = new Vector3(R[0, 2], R[1, 2], R[2, 2]);

        cam.transform.SetPositionAndRotation(pos, Quaternion.LookRotation(z_axis, -y_axis));
    }

    static Point3f GetDecartCoordsFromPanos(Point2f v, int squareRad = 1, int notValidValue = 0)
    {
        if (squareRad < 1)
            return new Point3f(0, 0, 0);

        var y = Mathf.RoundToInt(v.Y);
        var x = Mathf.RoundToInt(v.X);

        var sumDepth = 0f;
        var validPoints = 0;
        for (int i = Math.Max(y - squareRad, 0); i <= Math.Min(y + squareRad, EnvDataFields.SphereDepthPano.Height - 1); i++)
            for (int j = Math.Max(x - squareRad, 0); j <= Math.Min(x + squareRad, EnvDataFields.SphereDepthPano.Width - 1); j++)
            {
                var val = EnvDataFields.SphereDepthPano.Get<Vec3w>(i, j)[0];
                if (val != notValidValue)
                {
                    sumDepth += val;
                    validPoints++;
                }
            }

        if (validPoints == 0)
            return new Point3f(0, 0, 0);

        var radius = (sumDepth / validPoints) / 10f;
        var polarCoords = new Vector3(radius, 2 * Mathf.PI * v.X / EnvDataFields.SpherePano.Width, Mathf.PI * (EnvDataFields.SpherePano.Height - v.Y) / EnvDataFields.SpherePano.Height);
        return new Point3f(polarCoords.x * Mathf.Sin(polarCoords.y) * Mathf.Cos(polarCoords.z), polarCoords.x * Mathf.Sin(polarCoords.y) * Mathf.Sin(polarCoords.z), polarCoords.x * Mathf.Cos(polarCoords.z));
    }
}
