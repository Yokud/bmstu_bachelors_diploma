using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

public class KinectAdministrator : MonoBehaviour
{
    public Mesh Mesh;

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
    private bool KinectInitialized = false;

    Matrix4x4 kinectToWorld, flipMatrix;
    static KinectAdministrator instance;

    // Image stream handles for the kinect
    private IntPtr colorStreamHandle;
    private IntPtr depthStreamHandle;

    private int mapSize;

    // Depth data
    private Texture2D depthTexture;
    private Color32[] depthImage;
    private Rect depthRect;
    private ushort[] depthMap;

    // Color data
    private Texture2D ColorTexture;
    private Color32[] colorImage;
    private Rect colorRect;
    private byte[] colorMap;

    // returns the single KinectManager instance
    public static KinectAdministrator Instance => instance;

    // checks if Kinect is initialized and ready to use. If not, there was an error during Kinect-sensor initialization
    public static bool IsKinectInitialized()
    {
        return instance != null ? instance.IsInitialized() : false;
    }

    // checks if Kinect is initialized and ready to use. If not, there was an error during Kinect-sensor initialization
    public bool IsInitialized()
    {
        return KinectInitialized;
    }

    void Awake()
    {
        //CalibrationText = GameObject.Find("CalibrationText");
        int hr = 0;

        try
        {
            hr = KinectWrapper.NuiInitialize(KinectWrapper.NuiInitializeFlags.UsesSkeleton |
                KinectWrapper.NuiInitializeFlags.UsesDepthAndPlayerIndex | KinectWrapper.NuiInitializeFlags.UsesColor);
            if (hr != 0)
            {
                throw new Exception("NuiInitialize Failed");
            }

            depthStreamHandle = IntPtr.Zero;
            hr = KinectWrapper.NuiImageStreamOpen(KinectWrapper.NuiImageType.DepthAndPlayerIndex,
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

        // Initialize depth & label map related stuff
        mapSize = KinectWrapper.GetDepthWidth() * KinectWrapper.GetDepthHeight();
        depthTexture = new Texture2D(KinectWrapper.GetDepthWidth(), KinectWrapper.GetDepthHeight());
        depthImage = new Color32[mapSize];

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

        KinectInitialized = true;
    }

    // Make sure to kill the Kinect on quitting.
    void OnApplicationQuit()
    {
        if (KinectInitialized)
        {
            // Shutdown OpenNI
            KinectWrapper.NuiShutdown();
            instance = null;
        }
    }

    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        if (KinectInitialized)
        {
            // needed by the KinectExtras' native wrapper to check for next frames
            // uncomment the line below, if you use the Extras' wrapper, but none of the Extras' managers
            //KinectWrapper.UpdateKinectSensor();

            if (depthStreamHandle != IntPtr.Zero && KinectWrapper.PollDepth(depthStreamHandle, KinectWrapper.Constants.IsNearMode, ref depthMap))
            {
                UpdateDepthMap();
            }

            if (colorStreamHandle != IntPtr.Zero && KinectWrapper.PollColor(colorStreamHandle, ref colorMap, ref colorImage))
            {
                UpdateColorMap();
            }
        }

        // Kill the program with ESC.
        if (Input.GetKeyDown(KeyCode.Escape))
        {
            Application.Quit();
        }
    }

    void OnGUI()
    {
        if (KinectInitialized)
        {
            if (colorRect.width == 0 || ColorTexture.height == 0)
            {
                // get the main camera rectangle
                Rect cameraRect = Camera.main.pixelRect;

                // calculate map width and height in percent, if needed
                if (DisplayMapsWidthPercent == 0f)
                    DisplayMapsWidthPercent = (KinectWrapper.GetDepthWidth() / 2) * 100 / cameraRect.width;

                float displayMapsWidthPercent = DisplayMapsWidthPercent / 100f;
                float displayMapsHeightPercent = displayMapsWidthPercent * KinectWrapper.GetColorHeight() / KinectWrapper.GetColorWidth();

                float displayWidth = cameraRect.width * displayMapsWidthPercent;
                float displayHeight = cameraRect.width * displayMapsHeightPercent;

                colorRect = new Rect(cameraRect.width - displayWidth, cameraRect.height, displayWidth, -displayHeight);
            }

            GUI.DrawTexture(colorRect, ColorTexture);

            if (depthRect.width == 0 || depthRect.height == 0)
            {
                // get the main camera rectangle
                Rect cameraRect = Camera.main.pixelRect;

                // calculate map width and height in percent, if needed
                if (DisplayMapsWidthPercent == 0f)
                {
                    DisplayMapsWidthPercent = (KinectWrapper.GetDepthWidth() / 2) * 100 / cameraRect.width;
                }

                float displayMapsWidthPercent = DisplayMapsWidthPercent / 100f;
                float displayMapsHeightPercent = displayMapsWidthPercent * KinectWrapper.GetDepthHeight() / KinectWrapper.GetDepthWidth();

                float displayWidth = cameraRect.width * displayMapsWidthPercent;
                float displayHeight = cameraRect.width * displayMapsHeightPercent;

                depthRect = new Rect(cameraRect.width - displayWidth, cameraRect.height, displayWidth, -displayHeight);
            }

            GUI.DrawTexture(depthRect, depthTexture);
        }
    }

    // Update the User Map
    void UpdateDepthMap()
    {
        for (int i = 0; i < mapSize; i++)
        {
            // Flip the texture as we convert label map to color array
            int flipIndex = i; // usersMapSize - i - 1;

            float val = (KinectWrapper.GetRealDepth(depthMap[i]) - KinectWrapper.GetMinDepth()) / (float)KinectWrapper.DepthDelta();
            depthImage[flipIndex] = new Color(val, val, val);
        }

        // Draw it!
        depthTexture.SetPixels32(depthImage);
        depthTexture.Apply();
    }

    // Update the Color Map
    void UpdateColorMap()
    {
        ColorTexture.SetPixels32(colorImage);
        ColorTexture.Apply();
    }
}
