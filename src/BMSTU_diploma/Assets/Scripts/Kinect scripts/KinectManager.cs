using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UnityEngine.SceneManagement;
using UnityEngine.UI;

public class KinectManager : MonoBehaviour
{
    public GameObject PlaneGrid;
    public GameObject Background;

    // Kinect elevation angle (in degrees)
    public int SensorAngle = 0;

    // How high off the ground is the sensor (in meters).
    public float SensorHeight = 1.0f;

    // Public Float to specify the image width used by depth and color maps, as % of the camera width. the height is calculated depending on the width.
    // if percent is zero, it is calculated internally to match the selected width and height of the depth image
    public float DisplayMapsWidthPercent = 20f;

    // GUI Text to show messages.
    public UnityEngine.UI.Text CalibrationText;

    // Bool to keep track of whether Kinect has been initialized
    private bool kinectInitialized = false;

    Matrix4x4 kinectToWorld, flipMatrix;
    static KinectManager instance;

    // Image stream handles for the kinect
    private IntPtr colorStreamHandle;
    private IntPtr depthStreamHandle;

    private int mapSize;

    // Depth data
    private ushort[] depthMap;

    // Color data
    private Texture2D ColorTexture;
    private Color32[] colorImage;
    private Rect colorRect;
    private byte[] colorMap;

    // Mesh info
    Vector3[] newVertices;
    Vector3[] newNormals;
    int[] newTriangles;
    Mesh MyMesh;
    ushort[] FilterdAndCroppedDepthImage;
    float[] FloatValues;
    int WidthBuffer;
    int HeightBuffer;

    int Width = KinectWrapper.GetDepthWidth();
    int Height = KinectWrapper.GetDepthHeight();
    int MinDepthValue = KinectWrapper.GetMinDepth();
    int MaxDepthValue = KinectWrapper.GetMaxDepth();
    public float MeshHeigth;

    UnityEngine.UI.Image bg;

    // returns the single KinectManager instance
    public static KinectManager Instance => instance;

    public bool KinectInitialized => kinectInitialized;

    void Awake()
    {
        //CalibrationText = GameObject.Find("CalibrationText");
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

            // set kinect elevation angle
            KinectWrapper.NuiCameraElevationSetAngle(SensorAngle);

            //create the transform matrix that converts from kinect-space to world-space
            Quaternion quatTiltAngle = new Quaternion();
            quatTiltAngle.eulerAngles = new Vector3(-SensorAngle, 0.0f, 0.0f);

            //float heightAboveHips = SensorHeight - 1.0f;

            // transform matrix - kinect to world
            //kinectToWorld.SetTRS(new Vector3(0.0f, heightAboveHips, 0.0f), quatTiltAngle, Vector3.one);
            kinectToWorld.SetTRS(new Vector3(0.0f, SensorHeight, 0.0f), quatTiltAngle, Vector3.one);
            flipMatrix = Matrix4x4.identity;
            flipMatrix[2, 2] = -1;

            instance = this;
            DontDestroyOnLoad(gameObject);
        }
        catch (DllNotFoundException e)
        {
            string message = "Please check the Kinect SDK installation.";
            Debug.LogError(message);
            Debug.LogError(e.ToString());
            if (CalibrationText != null)
                CalibrationText.text = message;

            return;
        }
        catch (Exception e)
        {
            string message = e.Message + " - " + KinectWrapper.GetNuiErrorString(hr);
            Debug.LogError(message);
            Debug.LogError(e.ToString());
            if (CalibrationText != null)
                CalibrationText.text = message;

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
            CalibrationText.text = "Kinect initialized";
        }

        Debug.Log("Kinect initialized.");

        kinectInitialized = true;
    }

    // Make sure to kill the Kinect on quitting.
    void OnApplicationQuit()
    {
        if (kinectInitialized)
        {
            // Shutdown OpenNI
            KinectWrapper.NuiShutdown();
            instance = null;
        }
    }

    // Start is called before the first frame update
    void Start()
    {
        WidthBuffer = Width;
        HeightBuffer = Height;

        MyMesh = new Mesh
        {
            indexFormat = UnityEngine.Rendering.IndexFormat.UInt32
        };
        PlaneGrid.GetComponent<MeshFilter>().mesh = MyMesh;

        bg = Background.GetComponentInChildren<UnityEngine.UI.Image>();

        SetupArrays();
    }

    // Update is called once per frame
    void Update()
    {
        if (kinectInitialized)
        {
            // needed by the KinectExtras' native wrapper to check for next frames
            // uncomment the line below, if you use the Extras' wrapper, but none of the Extras' managers
            //KinectWrapper.UpdateKinectSensor();

            if (depthStreamHandle != IntPtr.Zero && KinectWrapper.PollDepth(depthStreamHandle, KinectWrapper.Constants.IsNearMode, ref depthMap))
            {
                CheckArrays();
                //FilterDepthMap();
                CalculateFloatValues();
                UpdateMesh();
            }

            if (colorStreamHandle != IntPtr.Zero && KinectWrapper.PollColor(colorStreamHandle, ref colorMap, ref colorImage))
            {
                UpdateColorMap();
                if (bg != null)
                    bg.sprite = Sprite.Create(ColorTexture, new Rect(0 ,0, Width, Height), new Vector2());
                //bg.canvas.App
                //Graphics.Blit(ColorTexture, Camera.main.targetTexture);
            }
        }

        // Kill the program with ESC.
        if (Input.GetKeyDown(KeyCode.Escape))
        {
            SceneManager.LoadScene("Menu");
        }
    }

    void CheckArrays()
    {
        if ((Width != WidthBuffer) || (Height != HeightBuffer))
        {
            SetupArrays();
            WidthBuffer = Width;
            HeightBuffer = Height;
        }
    }

    void SetupArrays()
    {
        FilterdAndCroppedDepthImage = new ushort[Width * Height];
        FloatValues = new float[Width * Height];
        newVertices = new Vector3[Width * Height];
        newNormals = new Vector3[Width * Height];
        newTriangles = new int[(Width - 1) * (Height - 1) * 6];

        for (int H = 0; H < Height; H++)
        {
            for (int W = 0; W < Width; W++)
            {
                int Index = GetArrayIndex(W, H);
                newVertices[Index] = new Vector3(W, H, 0f);
                newNormals[Index] = new Vector3(0, 0, 1);

                if ((W != (Width - 1)) && (H != (Height - 1)))
                {
                    int TopLeft = Index;
                    int TopRight = Index + 1;
                    int BotLeft = Index + Width;
                    int BotRight = Index + 1 + Width;

                    int TrinagleIndex = W + H * (Width - 1);
                    newTriangles[TrinagleIndex * 6 + 0] = TopLeft;
                    newTriangles[TrinagleIndex * 6 + 1] = BotLeft;
                    newTriangles[TrinagleIndex * 6 + 2] = TopRight;
                    newTriangles[TrinagleIndex * 6 + 3] = BotLeft;
                    newTriangles[TrinagleIndex * 6 + 4] = BotRight;
                    newTriangles[TrinagleIndex * 6 + 5] = TopRight;
                }
            }
        }

        MyMesh.Clear();
        MyMesh.vertices = newVertices;
        MyMesh.normals = newNormals;
        MyMesh.triangles = newTriangles;
    }

    void CalculateFloatValues()
    {
        for (int i = 0; i < mapSize; i++)
        {
            int ImageValue = KinectWrapper.GetRealDepth(depthMap[mapSize - i - 1]);

            //Clamp Value
            ImageValue = (ushort)Mathf.Clamp(ImageValue, MinDepthValue, MaxDepthValue);

            //Calculate
            float FloatValue = (ImageValue - MinDepthValue) / (float)(MaxDepthValue - MinDepthValue);
            FloatValues[i] = FloatValue;
        }
    }

    void UpdateMesh()
    {
        // надо добавить удаление полигонов с большим разрывом по глубине

        for (int i = 0; i < mapSize; i++)
            ProcessPixel(i);

        MyMesh.vertices = newVertices;
        MyMesh.RecalculateNormals();
    }

    void ProcessPixel(int i)
    {
        //Calc Position
        newVertices[i].z = FloatValues[i] * MeshHeigth;
    }

    int GetArrayIndex(int W, int H)
    {
        if ((W < 0) || (W >= Width) || (H < 0) || (H >= Height))
        {
            return -1;
        }

        return W + H * Width;
    }

    // Update the Color Map
    void UpdateColorMap()
    {
        ColorTexture.SetPixels32(colorImage);
        ColorTexture.Apply();
    }
}
