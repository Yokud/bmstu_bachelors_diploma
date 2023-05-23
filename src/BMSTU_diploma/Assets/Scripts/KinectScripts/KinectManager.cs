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

    // Kinect elevation angle (in degrees)
    public int SensorAngle = 0;

    // How high off the ground is the sensor (in meters).
    public float SensorHeight = 1.0f;

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
    Mesh MyMesh;
    float[] NormalizedDepthValues;
    const float verticesDepthDeltaTreshold = 0.15f;

    readonly int Width = KinectWrapper.GetDepthWidth();
    readonly int Height = KinectWrapper.GetDepthHeight();
    readonly int MinDepthValue = KinectWrapper.GetMinDepth();
    readonly int MaxDepthValue = KinectWrapper.GetMaxDepth();
    public float MeshHeight;

    Image bg;

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
                CalibrationText.text = "Status: " + message;
            }
            return;
        }
        catch (Exception e)
        {
            string message = "Status: " + e.Message + " - " + KinectWrapper.GetNuiErrorString(hr);
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
            CalibrationText.text = "Status: " + "Kinect initialized";
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
        MyMesh = new Mesh
        {
            indexFormat = UnityEngine.Rendering.IndexFormat.UInt32
        };
        PlaneGrid.GetComponent<MeshFilter>().mesh = MyMesh;

        bg = Background.GetComponentInChildren<Image>();

        cam = Camera.main;

        GetFrustumParams(Background.transform.position.z, out float bgHeight, out float bgWidth);
        var rt = Background.GetComponent<RectTransform>();
        rt.sizeDelta = new Vector2(bgWidth, bgHeight);

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
                    bg.sprite = Sprite.Create(ColorTexture, new Rect(0 ,0, Width, Height), new Vector2());
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

        MyMesh.Clear();
        MyMesh.vertices = newVertices;
        MyMesh.normals = newNormals;
        MyMesh.triangles = newTriangles;
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
        var cameraToWorldMatrix = cam.cameraToWorldMatrix;
        var projMatrixInverse = cam.projectionMatrix.inverse;
        Parallel.For(0, mapSize, (i) =>
        {
            var (w, h) = GetScreenCoords(i);
            //newVertices[i] = cam.ScreenToWorldPoint(new Vector3(w, h, NormalizedDepthValues[i] * MeshHeight + planeGridZ));
            newVertices[i] = ManualScreenToWorldPoint(new Vector2(w, h), NormalizedDepthValues[i] * MeshHeight + planeGridZ, cameraToWorldMatrix, projMatrixInverse);
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

        MyMesh.vertices = newVertices;
        MyMesh.triangles = newTriangles;
        MyMesh.RecalculateNormals();
    }

    public Vector3 ManualScreenToWorldPoint(Vector2 screenPoint, float distance, Matrix4x4 cameraToWorldMatrix, Matrix4x4 projectionMatrixInverse)
    {
        // here we are converting screen point in screen space to camera space point placed on a plane "distance" away from the camera
        // screen point is in range [(0,0) - (Screen.Width, Screen.Height)]
        Vector2 pointViewportSpace = screenPoint / new Vector2(Width, Height); // convert space [(0,0) - (Screen.Width, Screen.Height)] to [(0,0) - (1,1)]
        Vector2 pointCameraSpaceNormalized = (pointViewportSpace * 2.0f) - Vector2.one; // convert space [(0,0) - (1,1)] to [(-1,-1) - (1,1)]
        Vector2 pointCameraSpace = pointCameraSpaceNormalized * distance; // convert space [(-1,-1) - (1,1)] to [(-dist,-dist) - (dist, dist)]
        Vector4 planePoint = new(pointCameraSpace.x, pointCameraSpace.y, distance, distance); // define the point (don't know why z and w components need to be set to distance)

        // calculate convertion matrix from camera space to world space
        Matrix4x4 matrix = cameraToWorldMatrix * projectionMatrixInverse;
        // multiply world point by VP matrix
        Vector4 worldPoint = matrix * planePoint;

        return worldPoint;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    int GetArrayIndex(int W, int H)
    {
        if ((W < 0) || (W >= Width) || (H < 0) || (H >= Height))
        {
            return -1;
        }

        return W + H * Width;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    (int w, int h) GetScreenCoords(int index)
    {
        return (index % Width, index / Width);
    }

    // Update the Color Map
    void UpdateColorMap()
    {
        ColorTexture.SetPixels32(colorImage);
        ColorTexture.Apply();
    }

    void UpdateCameraOrientation()
    {

    }
}
