// comment or uncomment the following #define directives
// depending on whether you use KinectExtras together with KinectManager

//#define USE_KINECT_INTERACTION_OR_FACETRACKING
//#define USE_SPEECH_RECOGNITION
	
using UnityEngine;

using System;
using System.Collections;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using System.Runtime.CompilerServices;
using System.IO;
using System.Text; 

// Wrapper class that holds the various structs and dll imports
// needed to set up a model with the Kinect.
public class KinectWrapper
{
	public static class Constants
	{	
		public const float NuiDepthHorizontalFOV = 58.5f;
		public const float NuiDepthVerticalFOV = 45.6f;
		
		public const int ColorImageWidth = 640;
		public const int ColorImageHeight = 480;
		public const NuiImageResolution ColorImageResolution = NuiImageResolution.resolution640x480;
		
		public const int DepthImageWidth = 640;
		public const int DepthImageHeight = 480;
		public const NuiImageResolution DepthImageResolution = NuiImageResolution.resolution640x480;
		
		public const bool IsNearMode = false;

		public const int MinDefaultDepth = 800;
		public const int MaxDefaultDepth = 4000;

        public const int MinNearModeDepth = 400;
        public const int MaxNearModeDepth = 3000;

		public const int DepthRightShift = 3;
    }
	
	/// <summary>
	///Structs and constants for interfacing C# with the Kinect.dll 
	/// </summary>

    [Flags]
    public enum NuiInitializeFlags : uint
    {
		UsesAudio = 0x10000000,
        UsesDepthAndPlayerIndex = 0x00000001,
        UsesColor = 0x00000002,
        UsesSkeleton = 0x00000008,
        UsesDepth = 0x00000020,
		UsesHighQualityColor = 0x00000040
    }
	
	public enum NuiErrorCodes : uint
	{
		FrameNoData = 0x83010001,
		StreamNotEnabled = 0x83010002,
		ImageStreamInUse = 0x83010003,
		FrameLimitExceeded = 0x83010004,
		FeatureNotInitialized = 0x83010005,
		DeviceNotGenuine = 0x83010006,
		InsufficientBandwidth = 0x83010007,
		DeviceNotSupported = 0x83010008,
		DeviceInUse = 0x83010009,
		
		DatabaseNotFound = 0x8301000D,
		DatabaseVersionMismatch = 0x8301000E,
		HardwareFeatureUnavailable = 0x8301000F,
		
		DeviceNotConnected = 0x83010014,
		DeviceNotReady = 0x83010015,
		SkeletalEngineBusy = 0x830100AA,
		DeviceNotPowered = 0x8301027F,
	}
	
	public enum NuiImageType
	{
		DepthAndPlayerIndex = 0,	// USHORT
		Color,						// RGB32 data
		ColorYUV,					// YUY2 stream from camera h/w, but converted to RGB32 before user getting it.
		ColorRawYUV,				// YUY2 stream from camera h/w.
		Depth						// USHORT
	}
	
	public enum NuiImageResolution
	{
		resolutionInvalid = -1,
		resolution80x60 = 0,
		resolution320x240 = 1,
		resolution640x480 = 2,
		resolution1280x960 = 3     // for hires color only
	}
	
	public enum NuiImageStreamFlags
	{
		None = 0x00000000,
		SupressNoFrameData = 0x0001000,
		EnableNearMode = 0x00020000,
		TooFarIsNonZero = 0x0004000
	}
	
	public struct NuiImageViewArea
	{
	    public int eDigitalZoom;
	    public int lCenterX;
	    public int lCenterY;
	}
	
	public struct NuiImageFrame
	{
		public Int64 liTimeStamp;
		public uint dwFrameNumber;
		public NuiImageType eImageType;
		public NuiImageResolution eResolution;
		//[MarshalAsAttribute(UnmanagedType.Interface)]
		public IntPtr pFrameTexture;
		public uint dwFrameFlags_NotUsed;
		public NuiImageViewArea ViewArea_NotUsed;
	}
	
	public struct NuiLockedRect
	{
		public int pitch;
		public int size;
		//[MarshalAsAttribute(UnmanagedType.U8)] 
		public IntPtr pBits; 
		
	}
	
	public struct ColorCust
	{
		public byte b;
		public byte g;
		public byte r;
		public byte a;
	}
	
	public struct ColorBuffer
	{
		[MarshalAs(UnmanagedType.ByValArray, SizeConst = 640 * 480)]
		public ColorCust[] pixels;
	}
	
	public struct DepthBuffer
	{
		[MarshalAs(UnmanagedType.ByValArray, SizeConst = 640 * 480, ArraySubType = UnmanagedType.U2)]
		public ushort[] pixels;
	}
	
	public struct NuiSurfaceDesc
	{
		uint width;
		uint height;
	}
	
	[Guid("13ea17f5-ff2e-4670-9ee5-1297a6e880d1")]
	[InterfaceType(ComInterfaceType.InterfaceIsIUnknown)]
	[ComImport()]
	public interface INuiFrameTexture
	{
        [MethodImpl(MethodImplOptions.InternalCall, MethodCodeType = MethodCodeType.Runtime)]
        [PreserveSig]
        int BufferLen();

        [MethodImpl (MethodImplOptions.InternalCall, MethodCodeType=MethodCodeType.Runtime)]
		[PreserveSig]
		int Pitch();

		[MethodImpl (MethodImplOptions.InternalCall, MethodCodeType=MethodCodeType.Runtime)]
		[PreserveSig]
		int LockRect(uint Level,ref NuiLockedRect pLockedRect,IntPtr pRect, uint Flags);

        [MethodImpl(MethodImplOptions.InternalCall, MethodCodeType = MethodCodeType.Runtime)]
        [PreserveSig]
        int GetLevelDesc(uint Level, ref NuiSurfaceDesc pDesc);

        [MethodImpl (MethodImplOptions.InternalCall, MethodCodeType=MethodCodeType.Runtime)]
		[PreserveSig]
		int UnlockRect(uint Level);
	}
	
	/* 
	 * kinect NUI (general) functions
	 */
    [DllImport(@"Kinect10.dll", EntryPoint = "NuiInitialize")]
    public static extern int NuiInitialize(NuiInitializeFlags dwFlags);

	[DllImport(@"Kinect10.dll", EntryPoint = "NuiShutdown")]
    public static extern void NuiShutdown();
	
	[DllImport(@"Kinect10.dll", EntryPoint = "NuiCameraElevationSetAngle")]
	public static extern int NuiCameraElevationSetAngle(int angle);
	
	[DllImport(@"Kinect10.dll", EntryPoint = "NuiCameraElevationGetAngle")]
    public static extern int NuiCameraElevationGetAngle(out int plAngleDegrees);
	
	/*
	 * kinect video functions
	 */
	[DllImport(@"Kinect10.dll", EntryPoint = "NuiImageStreamOpen")]
    public static extern int NuiImageStreamOpen(NuiImageType eImageType, NuiImageResolution eResolution, uint dwImageFrameFlags_NotUsed, uint dwFrameLimit, IntPtr hNextFrameEvent, ref IntPtr phStreamHandle);
	
	[DllImport(@"Kinect10.dll", EntryPoint = "NuiImageStreamGetNextFrame")]
    public static extern int NuiImageStreamGetNextFrame(IntPtr phStreamHandle, uint dwMillisecondsToWait, ref IntPtr ppcImageFrame);
	
	[DllImport(@"Kinect10.dll", EntryPoint = "NuiImageStreamReleaseFrame")]
    public static extern int NuiImageStreamReleaseFrame(IntPtr phStreamHandle, IntPtr ppcImageFrame);
	
	[DllImport(@"Kinect10.dll", EntryPoint = "NuiImageStreamSetImageFrameFlags")]
	public static extern int NuiImageStreamSetImageFrameFlags (IntPtr phStreamHandle, NuiImageStreamFlags dvImageFrameFlags);
	
	public static string GetNuiErrorString(int hr)
	{
		string message = string.Empty;
		uint uhr = (uint)hr;
		
		switch(uhr)
		{
			case (uint)NuiErrorCodes.FrameNoData:
				message = "Frame contains no data.";
				break;
			case (uint)NuiErrorCodes.StreamNotEnabled:
				message = "Stream is not enabled.";
				break;
			case (uint)NuiErrorCodes.ImageStreamInUse:
				message = "Image stream is already in use.";
				break;
			case (uint)NuiErrorCodes.FrameLimitExceeded:
				message = "Frame limit is exceeded.";
				break;
			case (uint)NuiErrorCodes.FeatureNotInitialized:
				message = "Feature is not initialized.";
				break;
			case (uint)NuiErrorCodes.DeviceNotGenuine:
				message = "Device is not genuine.";
				break;
			case (uint)NuiErrorCodes.InsufficientBandwidth:
				message = "Bandwidth is not sufficient.";
				break;
			case (uint)NuiErrorCodes.DeviceNotSupported:
				message = "Device is not supported (e.g. Kinect for XBox 360).";
				break;
			case (uint)NuiErrorCodes.DeviceInUse:
				message = "Device is already in use.";
				break;
			case (uint)NuiErrorCodes.DatabaseNotFound:
				message = "Database not found.";
				break;
			case (uint)NuiErrorCodes.DatabaseVersionMismatch:
				message = "Database version mismatch.";
				break;
			case (uint)NuiErrorCodes.HardwareFeatureUnavailable:
				message = "Hardware feature is not available.";
				break;
			case (uint)NuiErrorCodes.DeviceNotConnected:
				message = "Device is not connected.";
				break;
			case (uint)NuiErrorCodes.DeviceNotReady:
				message = "Device is not ready.";
				break;
			case (uint)NuiErrorCodes.SkeletalEngineBusy:
				message = "Skeletal engine is busy.";
				break;
			case (uint)NuiErrorCodes.DeviceNotPowered:
				message = "Device is not powered.";
				break;
				
			default:
				message = "hr=0x" + uhr.ToString("X");
				break;
		}
		
		return message;
	}
	
	public static int GetDepthWidth()
	{
		return Constants.DepthImageWidth;
	}
	
	public static int GetDepthHeight()
	{
		return Constants.DepthImageHeight;
	}
	
	public static int GetColorWidth()
	{
		return Constants.ColorImageWidth;
	}
	
	public static int GetColorHeight()
	{
		return Constants.ColorImageHeight;
	}

	public static int GetMinDepth() => Constants.IsNearMode ? Constants.MinNearModeDepth : Constants.MinDefaultDepth;

	public static int GetMaxDepth() => Constants.IsNearMode ? Constants.MaxNearModeDepth : Constants.MaxDefaultDepth;

	public static int DepthDelta() => GetMaxDepth() - GetMinDepth();

	public static int GetRealDepth(int depth) => depth >> Constants.DepthRightShift;

    public static bool PollColor(IntPtr colorStreamHandle, ref byte[] videoBuffer, ref Color32[] colorImage)
	{
		IntPtr imageFramePtr = IntPtr.Zero;
		bool newColor = false;
	
		int hr = NuiImageStreamGetNextFrame(colorStreamHandle, 0, ref imageFramePtr);
		if (hr == 0)
		{
			newColor = true;
			
			NuiImageFrame imageFrame = (NuiImageFrame)Marshal.PtrToStructure(imageFramePtr, typeof(NuiImageFrame));
			INuiFrameTexture frameTexture = (INuiFrameTexture)Marshal.GetObjectForIUnknown(imageFrame.pFrameTexture);
			
			NuiLockedRect lockedRectPtr = new NuiLockedRect();
			IntPtr r = IntPtr.Zero;
			
			frameTexture.LockRect(0, ref lockedRectPtr, r, 0);
            ColorBuffer cb = (ColorBuffer)Marshal.PtrToStructure(lockedRectPtr.pBits, typeof(ColorBuffer));
            int totalPixels = Constants.ColorImageWidth * Constants.ColorImageHeight;

            for (int pix = 0; pix < totalPixels; pix++)
            {
                int ind = totalPixels - pix - 1;

                colorImage[ind].r = cb.pixels[pix].r;
                colorImage[ind].g = cb.pixels[pix].g;
                colorImage[ind].b = cb.pixels[pix].b;
                colorImage[ind].a = 255;
            }

			//// flip horizontally
   //         for (int j = 0; j < Constants.ColorImageHeight; j++)
   //         {
   //             int rowStart = 0;
   //             int rowEnd = Constants.ColorImageWidth - 1;

   //             while (rowStart < rowEnd)
   //             {
   //                 Color hold = colorImage[j * Constants.ColorImageWidth + rowStart];
   //                 colorImage[j * Constants.ColorImageWidth + rowStart] = colorImage[(j * Constants.ColorImageWidth) + rowEnd];
   //                 colorImage[j * Constants.ColorImageWidth + rowEnd] = hold;
   //                 rowStart++;
   //                 rowEnd--;
   //             }
   //         }

            frameTexture.UnlockRect(0);
			hr = NuiImageStreamReleaseFrame(colorStreamHandle, imageFramePtr);
		}
		
		return newColor;
	}
	
	public static bool PollDepth(IntPtr depthStreamHandle, bool isNearMode, ref ushort[] depthPlayerData)
	{
		IntPtr imageFramePtr = IntPtr.Zero;
		bool newDepth = false;

		if (isNearMode)
		{
			KinectWrapper.NuiImageStreamSetImageFrameFlags(depthStreamHandle, NuiImageStreamFlags.EnableNearMode | NuiImageStreamFlags.TooFarIsNonZero);
		}
		else
		{
			KinectWrapper.NuiImageStreamSetImageFrameFlags(depthStreamHandle, NuiImageStreamFlags.TooFarIsNonZero);
		}
		
		int hr = KinectWrapper.NuiImageStreamGetNextFrame(depthStreamHandle, 0, ref imageFramePtr);
		if (hr == 0)
		{
			newDepth = true;
			
			NuiImageFrame imageFrame = (NuiImageFrame)Marshal.PtrToStructure(imageFramePtr, typeof(NuiImageFrame));
			INuiFrameTexture frameTexture = (INuiFrameTexture)Marshal.GetObjectForIUnknown(imageFrame.pFrameTexture);
			
			NuiLockedRect lockedRectPtr = new NuiLockedRect();
			IntPtr r = IntPtr.Zero;
			
			frameTexture.LockRect(0, ref lockedRectPtr,r,0);

			DepthBuffer db = (DepthBuffer)Marshal.PtrToStructure(lockedRectPtr.pBits, typeof(DepthBuffer));
			depthPlayerData = db.pixels;

			frameTexture.UnlockRect(0);
			hr = KinectWrapper.NuiImageStreamReleaseFrame(depthStreamHandle, imageFramePtr);
		}
		
		return newDepth;
	}
}