// --------------------------------------------------------------------------------------------------------------------
// <copyright file="CameraManager.cs" company="FocalSpec Ltd">
// FocalSpec Ltd 2016-2019 
// </copyright>  
// <summary>
// Controls FocalSpec sensor.
// </summary>
// --------------------------------------------------------------------------------------------------------------------
  
using System;
using System.Collections.Concurrent; 
using System.Collections.Generic;       
using System.Linq;   
using System.Threading; 
using FocalSpec.FsApiNet.Model;
using FocalSpec.GuiExample.Model.Export; 

namespace FocalSpec.GuiExample.Model.Camera
{
    /// <summary>   
    /// Handles received profiles.
    /// </summary>
    /// <param name="profile">Profile to handle.</param>
    public delegate void PointCloudReceivedHandler(Profile profile); 

    /// <summary>
    /// Handles received raw images.
    /// </summary>
    /// <param name="rawImage">Raw image to handle.</param>
    public delegate void RawImageReceivedHandler(RawImage rawImage);

    /// <summary>
    /// Handles camera selection.
    /// </summary>
    /// <param name="cameraIds">List of camera ids.</param>
    public delegate void CameraSelectionHandler(List<string> cameraIds);

    /// <summary>
    /// Provides easy access to camera.
    /// </summary>
    public class CameraManager
    {
        /// <summary>
        /// The camera instance.
        /// </summary>
        private FsApi _camera;

        /// <summary>
        /// Settings of the sensor.
        /// </summary>
        private readonly ApplicationSettings _settings;

        /// <summary>
        /// Identifier for the camera.
        /// </summary>
        private string _cameraId;

        public string CameraId
        {
            set => _cameraId = value;  
        }

        /// <summary>
        /// Flag that is set to true, when the program is about to exist.
        /// </summary>
        private bool _exitRequested;

        /// <summary>
        /// The queue for the point clouds. 
        /// </summary>
        private readonly ConcurrentQueue<Profile> _queue = new ConcurrentQueue<Profile>();

        /// <summary>
        /// Gets or sets the camera version.
        /// </summary>
        public string CameraVersion { get; private set; }

        /// <summary>
        /// Gets or sets the serial number of camera, derived from MAC address
        /// </summary>
        public string CameraSn { get; private set; }

	    /// <summary>
	    /// Gets or sets the LCI sensor device serial number.
	    /// </summary>
	    public string DeviceSerialNumber { get; private set; }

        /// <summary>
        /// Gets the length of the local reception queue.
        /// </summary>
        public int QueueLength => _queue.Count;

	    /// <summary>
        /// Gets or sets the flag indicating whether the infinite queue size is enabled or not. If enabled, then queue size can cumulate over time.
        /// </summary>
        public bool IsInfiniteQueueSizeEnabled { get; set; }

        /// <summary>
        /// Gets the flag indicating whether the camera buffer (reception buffer inside FSAPI) is empty or not.
        /// </summary>
        public bool IsCameraBufferEmpty => !_queue.Any();

        /// <summary>
        /// Gets the flag indicating whether the HW supports Automatic Gain Control (AGC) or not.
        /// </summary>
        public bool IsAgcSupported
        {
            get
            {
                _camera.GetParameterSupport(_cameraId, SensorParameter.AgcEnabled, out bool agcSupport);
                return agcSupport;
            }
        }

        /// <summary>
        /// Gets the flag indicating whether Automatic Gain Control (AGC) is enabled or not.
        /// </summary>
        public bool IsAgcEnabled
        {
            get
            {
                if (!IsAgcSupported)
                    return false;
                return _isAgcEnabled;
            }
            private set { _isAgcEnabled = value; }
        }

        /// <summary>
        /// Gets the flag indicating whether camera image is vertically flipped.
        /// </summary>
        public bool IsFlippedImage { get; private set; }

        /// <summary>
        /// LED pulse width [µs] adjusted by the Automatic Gain Control (AGC).
        /// </summary>
        private float _agcAdjustedLedPulseWidth;

	    /// <summary>
	    /// Gets the flag indicating if sensor is High Speed camera or not.
	    /// </summary>
	    public bool IsHsCamera => _parameters.SensorType == 1220 || _parameters.SensorType == 1620;
        public bool IsXFilterSupported { get; set; }
        public bool IsHdrSupported { get; set; }
        public bool IsPulseWidthFloat => IsParameterSupported(SensorParameter.PulseWidthFloat);

        public bool ConnectionStatus { get; set; }
		
        public string RecipeFileName { get; set; }

        /// <summary>
        /// Sensor parameters.
        /// </summary>
        private readonly SensorParameterStore _parameters = SensorParameterStore.GetInstance();

        /// <summary>
        /// Last received profile header.
        /// </summary>
        private FsApi.Header _header;

        /// <summary>
        /// Gets the current LED pulse width in µs. If Automatic Gain Control (AGC) is in use, the value returned was adjusted by the AGC.
        /// </summary>
        public float CurrentLedPulseWidth
        {
            get
            {
                if (IsAgcSupported && IsAgcEnabled)
                {
                    return _agcAdjustedLedPulseWidth;
                }

                float pulseWidth;
                CameraStatusCode status;
                
                if (IsPulseWidthFloat)
                {
                    status = _camera.GetParameter(_cameraId, SensorParameter.PulseWidthFloat, out pulseWidth);
                }
                else
                {
                    status = _camera.GetParameter(_cameraId, SensorParameter.LedDuration, out int ledDuration);
                    pulseWidth = ledDuration;
                }

                if (status != CameraStatusCode.Ok)
                {
                    // Should never happen though.
                    throw new InvalidOperationException("Sensor connection issue.");
                }
                return pulseWidth;
            }
        }

        private int _selectedLayerIndex = 0;

		private bool _layerChanged = false;

	    public const int NotModified = -1;

        public int SelectedLayerIndex
        {
            get => _selectedLayerIndex;
            set
            {
	            if (value != _selectedLayerIndex)
	            {
		            _layerChanged = true;
	            }

	            _selectedLayerIndex = value;
            }
        }

        private ExportLayer _selectedLayer;
        private bool _isAgcEnabled;

        public ExportLayer SelectedLayer
        {
            get => this._selectedLayer;

	        set
            {
                if (Session.ViewMode == ViewMode.Recording || Session.ViewMode == ViewMode.RawImage)
                    return;

	            if (this._selectedLayer != value || _layerChanged)
		            this._selectedLayer = value;
	            else
	            {
		            return; 
	            }

                int wasGrabbing = 0; 
                if (Session.ViewMode == ViewMode.RealTime)
                { 
                    _camera.GetParameter(_cameraId, SensorParameter.Grabbing, out wasGrabbing);
                    if (wasGrabbing != 0) 
                    {
                        _camera.StopGrabbing(_cameraId);
                        Thread.Sleep(100);  
                    }
                }       
                  
                switch (this._selectedLayer)   
                {   
                    case ExportLayer.All:             
                        this._camera.RemoveLineCallback(this._cameraId);
                        _camera.SetProfileCallback(_cameraId, ProfileReceptionCallback);
                        break;      

                    case ExportLayer.Top: 
                        _camera.RemoveLineCallback(_cameraId); 
                        _camera.RemoveProfileCallback(_cameraId);
                        _camera.SetLineSortingOrder(_cameraId, SortingOrder.FromTopToBottom);
                        for (int i = 0; i <= SelectedLayerIndex; i++)
                            _camera.SetLineCallback(_cameraId, i, this.LineCallback);
                        break;

                    case ExportLayer.Bottom:
                        _camera.RemoveLineCallback(_cameraId);
                        _camera.RemoveProfileCallback(_cameraId); 
                        _camera.SetLineSortingOrder(_cameraId, SortingOrder.FromBottomToTop);
                        for (int i = 0; i <= SelectedLayerIndex; i++)
                            _camera.SetLineCallback(_cameraId, i, this.LineCallback);
                        break;

                    case ExportLayer.BrightestAndTop:
                        _camera.RemoveLineCallback(_cameraId);
                        _camera.RemoveProfileCallback(_cameraId);
                        _camera.SetLineSortingOrder(_cameraId, SortingOrder.FromMaxIntensityToLowerAndTopToBottom);
                        for (int i = 0; i <= SelectedLayerIndex; i++)
                            _camera.SetLineCallback(_cameraId, i, this.LineCallback);
                        break;
                    default: break;
                }

                if (Session.ViewMode == ViewMode.RealTime && wasGrabbing!=0)
                {
                    _camera.StartGrabbing(_cameraId);
                }

	            _layerChanged = false;
            }
        }

        public bool IntensityCalibrationInUse { get; set; }

        /// <summary>
		/// Event fires when a point cloud is received from the camera.
		/// </summary>
		public event PointCloudReceivedHandler OnPointCloudReceivedEvent;

        /// <summary>
        /// Event fires when a raw image is received from the camera.
        /// </summary>
        public event RawImageReceivedHandler OnRawImageReceivedEvent;

        /// <summary>
        /// Event handler for the camera selection.
        /// </summary>
        public event CameraSelectionHandler OnCameraSelectionEvent;

        /// <summary>
        /// Starts the reception thread.
        /// </summary>
        public CameraManager(ApplicationSettings settings)
        {
            _settings = settings;
            _cameraId = null;
            IsInfiniteQueueSizeEnabled = false;

            var queueThread = new Thread(QueueThread) { Priority = ThreadPriority.Highest };
            queueThread.Start();
        }

        /// <summary>
        /// Stop grabbing and close the camera instance.
        /// </summary>
        public void Close()
        {
            _exitRequested = true;

            if (string.IsNullOrEmpty(_cameraId))
            {
                return;
            }
            _camera.StopGrabbing(_cameraId);
            Thread.Sleep(500);
            _camera.Close();
        }

        private void LineCallback( 
            int layerId,
            float[] zValues,
            float[] intensityValues,
            int lineLength,
            double xStep, 
            FsApi.Header header)
        {
              
            if (_exitRequested)
            { 
                return;
            } 

            _header = header;
            Profile processed = new Profile(layerId, zValues, intensityValues, lineLength, xStep, header);
            _queue.Enqueue(processed);

            if (IsAgcSupported && IsAgcEnabled)
            {
                _agcAdjustedLedPulseWidth = header.PulseWidth;
            }

            if (!IsInfiniteQueueSizeEnabled && QueueLength > Defines.QueueMaxLength)
            {
                _queue.TryDequeue(out _);
            }  
            
        }
		
		/// <summary>
        /// Dummy example callback function for BatchCallback use.
        /// Loops all profiles from completed batch and sends them to application queue.
        /// </summary>
        /// <param name="layerId"></param>
        /// <param name="zValues"></param>
        /// <param name="intensityValues"></param>
        /// <param name="lineLength"></param>
        /// <param name="batchLength"></param>
        /// <param name="xStep"></param>
        /// <param name="lineHeaders"></param>
        private void BatchCallback( 
            int layerId,
            float[] zValues,
            float[] intensityValues,
            int lineLength,
            int batchLength,
            double xStep,
            IList<FsApi.Header> lineHeaders)
        {

            if (_exitRequested)
            {
                return;
            }

            var profileZvalues = new float[lineLength];
            var profileIntensityValues = new float[lineLength];
            var profileIndex = 0;

            foreach (var header in lineHeaders)
            {
                for (int i = 0; i < lineLength; i++)
                {
                   profileZvalues[i] = zValues[(profileIndex * lineLength) + i];
                   profileIntensityValues[i] = intensityValues[(profileIndex * lineLength) + i];
                }

                Profile processed = new Profile(layerId, profileZvalues, profileIntensityValues, lineLength, xStep, header);
                _queue.Enqueue(processed);
                profileIndex++;
            }

            if (!IsInfiniteQueueSizeEnabled && QueueLength > Defines.QueueMaxLength)
            {
                _queue.TryDequeue(out _);
            }
            
        }

        /// <summary>
        /// Enqueues profiles into an application buffer.
        /// </summary>
        /// <param name="profile">Profile received from the FSAPI.</param>
        /// <param name="header">Header for the point cloud.</param>
         private void ProfileReceptionCallback(IList<FsApi.Point> profile, FsApi.Header header)
         {
            if (_exitRequested)
            {
                return;
            }

            _header = header;

            try
            {
                Profile raw = new Profile(profile, header);
                Profile processed;

                switch (SelectedLayer)
                {
                    case ExportLayer.All:
                        processed = raw;
                        break;

                    default:
                        throw new ArgumentOutOfRangeException(string.Format("Unsupported layer {0}", SelectedLayer));
                }

                float average = 0;
                if (IsAgcEnabled)
                {
                    // AGC uses the entire signal regardless of filtering we do in the application.
                    if (raw.Points.Any())
                    {
                        foreach (var point in raw.Points)
                        {
                            average += point.Intensity;
                        }
                        average /= raw.Points.Count;
                    }
                }
                else
                {
                    if (processed.Points.Any())
                    {
                        foreach (var point in processed.Points)
                        {
                            average += point.Intensity;
                        }
                        average /= processed.Points.Count;
                    }
                }
                processed.AverageIntensity = average;

                _queue.Enqueue(processed);

                if (IsAgcSupported && IsAgcEnabled)
                {
                    _agcAdjustedLedPulseWidth = header.PulseWidth;
                }

                if (!IsInfiniteQueueSizeEnabled && QueueLength > Defines.QueueMaxLength)
                {
                    _queue.TryDequeue(out _);
                }
            }
            catch (Exception)
            {
                // ignored
            }
        }

        private void RawImageCallback(byte[] image, int bytes, FsApi.Header header)
        {
            if (_exitRequested)
                return;

            _header = header;
            header.PulseWidth = CurrentLedPulseWidth;

            OnRawImageReceivedEvent?.Invoke(new RawImage(image, bytes, header, _parameters.SensorWidth, IsFlippedImage));
        }

        /// <summary>
        /// Initializes camera parameters.
        /// </summary>
        /// <returns>Operation outcome.</returns>
        public CameraStatusCode InitializeCamera()
        {
            List<string> cameraIds;
			string cameraVersion, cameraSn;
            int cameraCount = _settings.ExpectedCameraCount;
            _header.ReceptionQueueSize = 0;

			if(_camera == null)  
				_camera = new FsApi();

            var cameraStatus = _camera.Open(ref cameraCount, out cameraIds, Defines.SensorDiscoveryTimeout);

            if (cameraStatus != CameraStatusCode.Ok)
                return cameraStatus;

            switch (cameraCount)
            {
                case 0:
                    _camera.Close();
                    return CameraStatusCode.NotConnected;
                case 1:
                    _cameraId = cameraIds.First();
                    break;
                default:
                    _cameraId = cameraIds.First(); 
                    OnCameraSelectionEvent?.Invoke(cameraIds); // _cameraId is changed by the event
                    break;
            }

            cameraStatus = _camera.Connect(_cameraId, _settings.IpAddress);
            bool selfTestFailed = cameraStatus == CameraStatusCode.CameraErrorSelftestGrabbingFailed;
            if (selfTestFailed)
            {
                cameraStatus = CameraStatusCode.Ok;
            }

		    if (cameraStatus != CameraStatusCode.Ok)
			    return cameraStatus;

            ConnectionStatus = true;

	        cameraStatus = _camera.GetDeviceVersion(_cameraId, out cameraVersion);
            CameraVersion = cameraVersion;
            if (cameraStatus != CameraStatusCode.Ok)
                return cameraStatus;

            cameraStatus = _camera.GetSerialNumber(_cameraId, out cameraSn);
            CameraSn = cameraSn;
            if (cameraStatus != CameraStatusCode.Ok)
                return cameraStatus;

	        cameraStatus = _camera.GetParameter(_cameraId, SensorParameter.SensorDataInFlash, out int calibrationsInCamera);
	        if (cameraStatus != CameraStatusCode.Ok) 
		        return cameraStatus;

	        cameraStatus = _camera.GetParameter(_cameraId, SensorParameter.DeviceSerialNumber, out string deviceSerialNumber);
	        DeviceSerialNumber = cameraStatus == CameraStatusCode.Ok ? deviceSerialNumber : "";
	        
            // Set maximum number of points per frame 
            _camera.SetParameter(_cameraId, SensorParameter.MaxPointCount, _parameters.MaxPointCount);

            var zCalibStatus = CameraStatusCode.CameraErrorSensorCalibrationFileNotSet;
            var xCalibStatus = CameraStatusCode.CameraErrorSensorCalibrationFileNotSet;

            if (_settings.ZCalibrationFile != null || _settings.XCalibrationFile != null)
	        {
		        zCalibStatus = _camera.SetParameter(_cameraId, SensorParameter.ZCalibrationFile, _settings.ZCalibrationFile);
                
		        if (zCalibStatus == CameraStatusCode.CameraErrorCalibFileAttributeNotFound ||
		            zCalibStatus == CameraStatusCode.CameraErrorCalibFileAttributeInvalid || 
		            zCalibStatus == CameraStatusCode.CameraErrorInvalidCalibrationFile)
		        {
			        // clear if not valid calibration.
			        _settings.ZCalibrationFile = null;
		        }

                xCalibStatus = _camera.SetParameter(_cameraId, SensorParameter.XCalibrationFile, _settings.XCalibrationFile);
		        
		        if (xCalibStatus == CameraStatusCode.CameraErrorCalibFileAttributeNotFound ||
			        xCalibStatus == CameraStatusCode.CameraErrorCalibFileAttributeInvalid || 
			        xCalibStatus == CameraStatusCode.CameraErrorInvalidCalibrationFile)
		        {
			        // clear if not valid calibration.
			        _settings.XCalibrationFile = null;
		        }
            }
            
            if (calibrationsInCamera == 1)
            {
                if (_settings.ZCalibrationFile == null)
                {
                    zCalibStatus = _camera.SetParameter(_cameraId, SensorParameter.ZCalibrationFile,
                        _settings.ZCalibrationFile);
                    if (zCalibStatus != CameraStatusCode.Ok)
                        return zCalibStatus;
                }

                if (_settings.XCalibrationFile == null)
                {
                    xCalibStatus = _camera.SetParameter(_cameraId, SensorParameter.XCalibrationFile,
                        _settings.XCalibrationFile);
                    if (xCalibStatus != CameraStatusCode.Ok)
                        return xCalibStatus;
                }
            }

            if (zCalibStatus  != CameraStatusCode.Ok)
                return zCalibStatus;

            if(xCalibStatus  != CameraStatusCode.Ok)
                return xCalibStatus;

	        cameraStatus = _camera.SetParameter(_cameraId, SensorParameter.PeakYUnit, Defines.PeakUnitMicrometer);
            if (cameraStatus != CameraStatusCode.Ok) 
                return cameraStatus; 
			
	        if (_settings.ZCalibrationFile == null)
            {	// read z calibration path from API and fill it to the settings 
	            cameraStatus = _camera.GetParameter(_cameraId, SensorParameter.ZCalibrationFile, out string zCalibrationFile);
	            if (cameraStatus != CameraStatusCode.Ok)
		            return cameraStatus;
                 
	            _settings.ZCalibrationFile = zCalibrationFile;
            }

            cameraStatus = _camera.SetParameter(_cameraId, SensorParameter.PeakXUnit, Defines.PeakUnitMicrometer);
            if (cameraStatus != CameraStatusCode.Ok)
                return cameraStatus;
             
			if (_settings.XCalibrationFile == null) 
		    {	// read x calibration path from API and fill it to the settings 
		        cameraStatus = _camera.GetParameter(_cameraId, SensorParameter.XCalibrationFile, out string xCalibrationFile);
		        if (cameraStatus != CameraStatusCode.Ok)
			        return cameraStatus;
	
		        _settings.XCalibrationFile = xCalibrationFile;
	        } 

	        cameraStatus = _camera.GetParameter(_cameraId, SensorParameter.SensorType, out int currentSensor);
            _parameters.SensorType = currentSensor; 
	        if (cameraStatus != CameraStatusCode.Ok)
		        return cameraStatus;

            // Use intensity calibration if the calibration file is found
            cameraStatus = _camera.SetParameter(_cameraId, SensorParameter.IntensityCorrectionEnable, 1);
            IntensityCalibrationInUse = (cameraStatus == CameraStatusCode.Ok);  
              
            /*
            IntensityCalibrationInUse = (_camera.SetParameter(_cameraId, SensorParameter.IntensityCalibrationFile,
                                        Path.ChangeExtension(_settings.ZCalibrationFile, ".intensity"))
                                    == CameraStatusCode.Ok);
            */
            ReloadSensorParameters();

            if (IsHsCamera && !_parameters.HdrEnabled)
            {
                _parameters.HdrKp1Pos = 95;
                _parameters.HdrVLow2 = 33;
            }

	        cameraStatus = SetWindowScale();
	        if (cameraStatus != CameraStatusCode.Ok)
	        {
		        return cameraStatus;
	        }

            // Due to case pin 2 is also used for triggering, disable source needs to be changed from 2 to 0
            // ignore error caused by sensors with old electronics
            /*
            cameraStatus = _camera.SetParameter(_cameraId, SensorParameter.TriggerSource, _parameters.TriggerSource);
            if (cameraStatus != CameraStatusCode.Ok &&  cameraStatus != CameraStatusCode.CameraErrorHwNotSupported)
                return cameraStatus;

            cameraStatus = _camera.SetParameter(_cameraId, SensorParameter.TriggerDisableSource, Defines.DefaultTriggerDisableSource);
            if (cameraStatus != CameraStatusCode.Ok &&  cameraStatus != CameraStatusCode.CameraErrorHwNotSupported)
                return cameraStatus;

            cameraStatus = _camera.SetParameter(_cameraId, SensorParameter.PulseDivider, _parameters.PulseDivider);
            if (cameraStatus != CameraStatusCode.Ok &&  cameraStatus != CameraStatusCode.CameraErrorHwNotSupported)
                return cameraStatus;
                        
            _camera.SetParameter(_cameraId, SensorParameter.PulseCurrentMiddle, _parameters.Current);
            _camera.SetParameter(_cameraId, SensorParameter.PulseCurrentEdges, _parameters.Current);
            cameraStatus = _camera.SetParameter(_cameraId, SensorParameter.Gain, !IsHsCamera ? _parameters.Gain : _parameters.GainHs);
            if (cameraStatus != CameraStatusCode.Ok)
                return cameraStatus;
		    */

            // Enable noise removal for the line callback
            //_camera.SetParameter(_cameraId, SensorParameter.NoiseRemoval, 1);

            // These are important to get reliable data transfer. 
            //  1.	Enable Jumbo Frames in the PC’s Ethernet adapter by setting the value 9014 Bytes for the Packet size
            //	2.	Set Receive Buffer Size in the PC Ethernet adapter to 2048

            // If you cannot enable jumbo frames set this to 1500
            if (!IsHsCamera)
	        {	// set parameters only if not high speed camera
		        cameraStatus = _camera.SetParameter(_cameraId, SensorParameter.Mtu, _parameters.JumboFrameMtu);
		        if (cameraStatus != CameraStatusCode.Ok)
			        return cameraStatus;

		        // Set Inter packet Delay, if needed. By default, it's 20.
		        cameraStatus = _camera.SetParameter(_cameraId, SensorParameter.Ifg, 20);
		        if (cameraStatus != CameraStatusCode.Ok)
			        return cameraStatus;
	        }

            /*
            // Set 70 um for first 6 layers
            for (int i = 0; i < 6;i++)
                _camera.SetLayerParameter(_cameraId, i, SensorParameter.LayerMaxThickness, 70.0f);

            // Use detection filter for 2nd and 3rd layers
            _camera.SetLayerParameter(_cameraId, 1, SensorParameter.LayerIntensityType, 1);
            _camera.SetLayerParameter(_cameraId, 2, SensorParameter.LayerIntensityType, 1);
            */

            /*
            _camera.SetParameter(_cameraId, SensorParameter.DetectMissingFirstLayer, 100.0);
            _camera.SetParameter(_cameraId, SensorParameter.DetectMissingFirstLayerX, 100.0);
            _camera.SetParameter(_cameraId, SensorParameter.DetectMissingFirstLayerLength, 2500.0);
            */

            /*  
            cameraStatus = _camera.SetParameter(_cameraId, SensorParameter.ResampleLineXResolution, 20.0);
            if (cameraStatus != CameraStatusCode.Ok) 
                return cameraStatus;

            cameraStatus = _camera.SetParameter(_cameraId, SensorParameter.AverageZFilterSize, 20.0);
            if (cameraStatus != CameraStatusCode.Ok)
                return cameraStatus;
            cameraStatus = _camera.SetParameter(_cameraId, SensorParameter.AverageIntensityFilterSize, 40.0);
            if (cameraStatus != CameraStatusCode.Ok)
                return cameraStatus;
            */

            // This is sample code section for dynamic sensor control parameter functionality
            /* <example> 

	        // Initialize dynamic parameter structure array up to 10 parameter sets
	        FsApi.DynSensorControl[] myDynamicSetting = 
	        {	// encoder location, image height, image offset, led pulse width
				new FsApi.DynSensorControl(1,     700, 294, 8.0f),
		        new FsApi.DynSensorControl(10000, 600, 244, 2.0f),
		        new FsApi.DynSensorControl(20000, 500, 294, 5.0f),
		        new FsApi.DynSensorControl(30000, 400, 344, 10.0f),
		        new FsApi.DynSensorControl(40000, 300, 394, 20.0f),
		        new FsApi.DynSensorControl(50000, 600, 244, 40.0f)
	        };
	        // Initialize version 2 of dynamic parameter structure array
			// if Threshold is included, supported only by HS CAM
	        FsApi.DynSensorControl[] newDynamicSetting = 
	        {	// encoder location, image height, image offset, led pulse width, Threshold
			    new FsApi.DynSensorControl(1,     700, 350, 8.0f,    24),
		        new FsApi.DynSensorControl(10000, 600, 400, 20.0f,   20),
		        new FsApi.DynSensorControl(20000, 500, 450, 50.0f,   25),
		        new FsApi.DynSensorControl(30000, 400, 500, 80.0f,   30),
		        new FsApi.DynSensorControl(40000, 300, 550, 100.0f,  20),
		        new FsApi.DynSensorControl(60000, 200, 600, 30.0f,   25)
	        };

	        // pass parameters to the camera
            cameraStatus = _camera.DynamicSensorCtrlParameters(_cameraId, newDynamicSetting.Length, newDynamicSetting);
	        // cameraStatus = _camera.DynamicSensorCtrlParameters(_cameraId, myDynamicSetting.Length, myDynamicSetting);
	        
	        // enable functionality
            cameraStatus = _camera.SetParameter(_cameraId, SensorParameter.DynSensorCtrlEnable, 1);
			// cameraStatus not checked feature might not be supported
	        // </example> */
            /*
	        cameraStatus = _camera.SetProfileCallback(_cameraId, ProfileReceptionCallback);
	        if (cameraStatus != CameraStatusCode.Ok)
		        return cameraStatus;
            */

            if (IsAgcSupported)
            {
                cameraStatus = EnableAgc(_parameters.IsAgcEnabled);
                switch (cameraStatus)
                {
                    case CameraStatusCode.CameraErrorHwNotSupported:
                    case CameraStatusCode.CameraErrorFirmwareNotSupporting:
                        break;
                    case CameraStatusCode.Ok:
                        if (_parameters.IsAgcEnabled)
                        {
                            cameraStatus = SetAgcTargetIntensity(_parameters.AgcTargetIntensity);
                            if (cameraStatus != CameraStatusCode.Ok)
                                return cameraStatus;
                        }

                        break;
                    default:
                        return cameraStatus;
                }
            }

            cameraStatus =_camera.GetParameter(_cameraId, SensorParameter.FlipXEnabled, out int flip);
            IsFlippedImage = flip == 1;
            //cameraStatus = _camera.StartGrabbing(_cameraId);

            if (selfTestFailed)
            {
                return CameraStatusCode.CameraErrorSelftestGrabbingFailed;
            }

		    return cameraStatus;
	    }
        
        /// <summary>
        /// Starts profile data acquisition.
        /// </summary>
        /// <param name="timeToSleep">Defines time to sleep [ms] before start grabbing. Is needed when sensor parameters are adjusted and grabbing stared right after </param>
        /// <returns> 0 if successful, any other positive value if failed or -1 if not performed because camera was already grabbing.</returns>
        public int StartGrabbing(int timeToSleep)
        {
            var status = (int) _camera.GetParameter(_cameraId, SensorParameter.Grabbing, out int isGrabbing);

	        if (timeToSleep > 0)
		        Thread.Sleep(timeToSleep); 

	        if (status != 0) 
	        {
		        return status; 
	        } 

	        if (isGrabbing == 1)
	        {
		        status = NotModified;
	        }
	        else
	        {
		        status = (int)_camera.StartGrabbing(_cameraId);
	        }

	        return status;
        }

        /// <summary>
        /// Stops profile data acquisition.
        /// </summary>
        /// <param name="timeToSleep">Defines time to sleep [ms] after stop grabbing. Is needed when sensor parameters are adjusted and grabbing stared right after </param>
        /// <returns> 0 if successful, any other positive value if failed or -1 if not performed because camera was not grabbing.</returns>
        public int StopGrabbing(int timeToSleep)
        {
	        var status = (int) _camera.GetParameter(_cameraId, SensorParameter.Grabbing, out int isGrabbing);

	        if (status != 0)
	        {
		        return status;
	        }

	        if (isGrabbing == 0)
	        {
		        status = NotModified;
	        }
	        else
	        {
		        status  = (int)_camera.StopGrabbing(_cameraId); 
	        }

	        if (timeToSleep > 0)
				Thread.Sleep(timeToSleep);
            
	        return status;
        }

        /// <summary>
        /// Queue thread. Check if there are data available in the queue. If there is, get the
        /// latest point cloud and call the event handlers.
        /// </summary>
        private void QueueThread()
        {
            // Loop until the program quits.
            while (!_exitRequested)
            {
                try
                {
                    Profile profile = null;

                    if (_queue.Any())
                    {
                        if (!_queue.TryDequeue(out profile))
                        {
                            Thread.Sleep(0);
                        }
                    }
                    else
                    {
                        Thread.Sleep(1);
                    }

                    try
                    {
                        if (OnPointCloudReceivedEvent != null && profile != null)
                        {
                            OnPointCloudReceivedEvent(profile);
                        }
                    }
                    catch
                    {
                        // ignored
                    }
                }
                catch
                {
                    // ignored
                }
            }
        }

        /// <summary>
        /// Sets frequency to the camera.
        /// </summary>
        /// <param name="freq">Sampling frequency [hz]. For external triggering, this will be the intended frequency, which is used to set up ROI.</param>
        /// <param name="isExternalPulsingEnabled">If true, 0 (value for external) will be written to sensor.</param>
        /// <returns>Status code indicating operation outcome.</returns>
        public CameraStatusCode SetFreq(int freq, bool isExternalPulsingEnabled)
        { 
		    var cameraStatus = CameraStatusCode.Ok;
                
			// nothing to do here is frequency has not been defined 
	        if (freq <= 0)	
			{    
				return cameraStatus; 
			}

	        // do not set delay less than 100 ms, otherwise sensor height setting does fail 
	        var grabbingStatus = StopGrabbing(110);

            // Previous implementation for calculating imaging height and offset replaced by this interface function.
            // HeightZeroAdjustEnabled is used for offset setting automatically. 
            int operation = (int) (_parameters.OffsetY < 0 ? RoiOperation.SetRoiForFps : RoiOperation.SetRoiForFpsCentered);
		    _camera.AdjustRoiAndFps(_cameraId, operation, freq, out _);
   
            if (Session.ViewMode == ViewMode.RawImage)
	        {   // do not use high frequencies while viewing raw image
		        freq = freq < 10 ? freq: 10;
		        isExternalPulsingEnabled = false;
	        }

	        if (isExternalPulsingEnabled)
	        {
		        freq = Defines.ExternalTriggering;
	        }

			cameraStatus = _camera.SetParameter(_cameraId, SensorParameter.PulseFrequency, isExternalPulsingEnabled ? 0 : freq);
		    
			if(grabbingStatus != NotModified)
				StartGrabbing(20);
		
		    return cameraStatus;
        }

        /// <summary>
        /// Sets LED pulse width. Utilizes Automatic Gain Control (AGC) if it is supported by the HW.
        /// </summary>
        /// <param name="us">LED pulse width in µs. If AGC is available, this is used as starting reference.</param>
        /// <param name="maxUs">If AGC is available, this is the max. pulse width AGC can adjust into. Otherwise this is null.</param>
        /// <exception cref="ArgumentException">Start LED pulse is higher than max. LED pulse.</exception>
        /// <returns>Status code indicating operation outcome.</returns>
        public CameraStatusCode SetPulseWidth(float us, int? maxUs)
        {
            CameraStatusCode status;

	        int grabbingStatus = StopGrabbing(20);

            if (maxUs.HasValue && IsAgcEnabled)
            {
                if (us > maxUs.Value)
                    throw new ArgumentException("LED pulse start reference must be smaller than LED pulse upper limit (max).");

                status = _camera.SetParameter(_cameraId, SensorParameter.AgcWiLimit, maxUs.Value);

                if (status != CameraStatusCode.Ok)
                {
	                if(grabbingStatus != NotModified)
						StartGrabbing(20);
                    return status;
                }
            }

            status = IsPulseWidthFloat ? _camera.SetParameter(_cameraId, SensorParameter.PulseWidthFloat, us) : _camera.SetParameter(_cameraId, SensorParameter.LedDuration, (int)us);
            
	        if(grabbingStatus != NotModified)
				StartGrabbing(20);  
            return status;
        }
         
        /// <summary>
        /// Enables/disables AGC.
        /// </summary>
        /// <param name="isEnabled">If true, tries to enable AGC.</param>
        /// <returns>CameraStatusCode.Ok if AGC is supported and enabled.</returns>
        public CameraStatusCode EnableAgc(bool isEnabled)
        {
	        if (IsAgcEnabled == isEnabled || (Session.ViewMode == ViewMode.RawImage))
		        return CameraStatusCode.Ok;

	        int grabbingStatus = StopGrabbing(30);
             
            CameraStatusCode status = _camera.SetParameter(_cameraId, SensorParameter.AgcEnabled, isEnabled ? 1 : 0);

            if (status == CameraStatusCode.Ok)
                IsAgcEnabled = isEnabled;

	        if(grabbingStatus != NotModified)
		        StartGrabbing(20);

            return status;
        }

        /// <summary>
        /// Sets AGC target intensity in greyscale.
        /// </summary>
        /// <param name="targetIntensity">Target intensity [0.0-255.0]</param>
        /// <returns>Status code indicating operation outcome.</returns>
        public CameraStatusCode SetAgcTargetIntensity(float targetIntensity)
        {
	        if(Session.ViewMode == ViewMode.RawImage)
				return CameraStatusCode.Ok;

	        int grabbingStatus = StopGrabbing(30);
            var status = _camera.SetParameter(_cameraId, SensorParameter.AgcTarget, targetIntensity);
           
	        if(grabbingStatus != NotModified)
				StartGrabbing(20);

            return status;
        }

	    /// <summary>
	    /// Sets calibrated axis offset and width in pixels.
	    /// </summary>
	    /// <returns>Status code indicating operation outcome.</returns>
	    public CameraStatusCode SetWindowScale()
        {
			// include check if supported by FW
		    _camera.SetLineSortingOrder(_cameraId, SortingOrder.FromTopToBottom);
		    _camera.SetLineCallback(_cameraId, 0, LineCallback);

		    _settings.OpticalProfileMinX = 0;
	
		    var cameraStatus = _camera.GetParameter(_cameraId, SensorParameter.Width, out int width);
		    if (cameraStatus != CameraStatusCode.Ok)
		    {
			    return cameraStatus;
		    }
            _parameters.SensorWidth = width;

		    cameraStatus = _camera.GetParameter(_cameraId, SensorParameter.LineCallbackXRes, out float piXRes);
		    if (cameraStatus != CameraStatusCode.Ok)
		    {
			    return cameraStatus;
		    }
            _parameters.AveragePixelWidth = piXRes * 0.001;
		    _settings.OpticalProfileMaxX = _parameters.AveragePixelWidth * width;
		    
			// not yet used
		    cameraStatus = _camera.GetParameter(_cameraId, SensorParameter.OffsetX, out int offsetX);
            _parameters.XOffset = offsetX;

		    _camera.RemoveLineCallback(_cameraId);

			return cameraStatus;
	    }
		
		/// <summary>
        /// Sets fir length.
        /// </summary>
        /// <param name="firLength">Fir Length [2,4,8,16]</param>
        /// <returns>Status code indicating operation outcome.</returns>
        public CameraStatusCode SetFirLength(int firLength)
        {
	        if(Session.ViewMode == ViewMode.RawImage)
		        return CameraStatusCode.Ok;

	        int grabbingStatus = StopGrabbing(30);
            var status = _camera.SetParameter(_cameraId, SensorParameter.FirLength, firLength);
	        if(grabbingStatus != NotModified)
				StartGrabbing(20);
            return status;
        }

        /// <summary>
        /// Sets averaging fir length.
        /// </summary>
        /// <param name="averagingFirLength">Averaging Fir Length [2,4,8,16]</param>
        /// <returns>Status code indicating operation outcome.</returns>
        public CameraStatusCode SetAveragingFirLength(int averagingFirLength)
        {
	        if(Session.ViewMode == ViewMode.RawImage)
		        return CameraStatusCode.Ok;
             
	        int grabbingStatus = StopGrabbing(30);
            var status = _camera.SetParameter(_cameraId, SensorParameter.FirAverLength, averagingFirLength);
	        if(grabbingStatus != NotModified)
				StartGrabbing(20);
            return status;
        }

        /// <summary>
        /// Sets peak core threshold.
        /// </summary>
        /// <param name="threshold">Peak core threshold [0-255]</param>
        /// <returns>Status code indicating operation outcome.</returns>
        public CameraStatusCode SetThreshold(int threshold)
        {
	        if(Session.ViewMode == ViewMode.RawImage)
		        return CameraStatusCode.Ok;

	        int grabbingStatus = StopGrabbing(30);
            var status = _camera.SetParameter(_cameraId, SensorParameter.PeakThreshold, threshold);
	        if(grabbingStatus != NotModified)
				StartGrabbing(20);
            return status;
        }

	    /// <summary>
	    /// Checks if individual parameter is supported by FSSDK, harware and firmware.
	    /// </summary>
	    /// <param name="parameterName">Name of sensor parameter</param>
	    /// <returns>Status code indicating operation outcome.</returns>
	    public bool IsParameterSupported(string parameterName)
	    {
		    var status = _camera.GetParameterSupport(_cameraId, parameterName, out var isSupported);
		    return isSupported;
	    }

        /// <summary>
        /// Sets raw image mode
        /// </summary>
        /// <param name="rawImageMode">Raw image mode on off</param>
        /// <param name="freq">Sampling frequency [hz]. For external triggering, this will be the intended frequency, which is used to set up ROI.</param>
        /// <param name="isExternalPulsingEnabled">If true, 0 (value for external) will be written to sensor.</param>
        /// <returns>Status code indicating operation outcome.</returns>
        public CameraStatusCode SetRawImageMode(bool rawImageMode, int freq, bool isExternalPulsingEnabled)
        {
            var grabbingStatus = StopGrabbing(100);
            CameraStatusCode status = CameraStatusCode.Ok;
            if (rawImageMode)
            {
	            Session.ViewMode = ViewMode.RawImage;
	            if (_selectedLayer == ExportLayer.All)
	            {
		            _camera.RemoveProfileCallback(_cameraId);     
	            }
	            else
	            {
		            _camera.RemoveLineCallback(_cameraId);
	            }
	            status = _camera.SetParameter(_cameraId, SensorParameter.PeakEnabled, 0);  
	            if (status == CameraStatusCode.Ok)
	            {
                    _parameters.IsPeakEnabled = false;
		            _camera.SetRawImageCallback(_cameraId, RawImageCallback);
	            }
            }
            else
            {
	            Session.ViewMode = ViewMode.RealTime;
                _camera.RemoveRawImageCallback(_cameraId);
                status = _camera.SetParameter(_cameraId, SensorParameter.PeakEnabled, 1);
                _parameters.IsPeakEnabled = true;

	            if (_selectedLayer == ExportLayer.All)
	            {
		            _camera.SetProfileCallback(_cameraId, ProfileReceptionCallback);     
	            }
	            else
	            {
                    for (int i = 0; i <= SelectedLayerIndex; i++)
		                _camera.SetLineCallback(_cameraId, i, LineCallback);
	            }
            }

	        EnableAgc(_parameters.IsAgcEnabled & !rawImageMode);

	        if (status == CameraStatusCode.Ok)
		        status = SetFreq(freq, isExternalPulsingEnabled);

            if(grabbingStatus != NotModified)
                StartGrabbing(20);
            return status;
        }

        /// <summary>
        /// Sets High dynamic range (HDR) for camera.
        /// </summary>
        /// <param name="hdrEnabled">Whether HDR is enabled</param>
        /// <param name="vLow2">vLow2</param>
        /// <param name="vLow3">vLow3</param>
        /// <param name="kp1Pos">kp1Pos</param>
        /// <param name="kp2Pos">kp2Pos</param>
        /// <returns>Status code indicating operation outcome.</returns>
        public CameraStatusCode SetHdr(bool hdrEnabled, float vLow2, float vLow3, float kp1Pos, float kp2Pos)
        {
            var status = CameraStatusCode.Ok;
            if (hdrEnabled)
            {
                if (!IsHsCamera)
                {
                    // Does HDR need disabling in standard sensor? Assuming yes because it's done here. In HS sensor it's not needed.
                    /// @todo Remove disabling HDR if not required in standard sensor.
                    status = _camera.SetParameter(_cameraId, SensorParameter.HdrEnabled, 0);
                }
                if (status == CameraStatusCode.Ok)
                    status = _camera.SetParameter(_cameraId, SensorParameter.VLow2Float, vLow2);
                if (status == CameraStatusCode.Ok && !IsHsCamera)
                    status = _camera.SetParameter(_cameraId, SensorParameter.VLow3Float, vLow3);
                if (status == CameraStatusCode.Ok)
                    status = _camera.SetParameter(_cameraId, SensorParameter.Kp1PosFloat, kp1Pos);
                if (status == CameraStatusCode.Ok && !IsHsCamera)
                    status = _camera.SetParameter(_cameraId, SensorParameter.Kp2PosFloat, kp2Pos);
                status = _camera.SetParameter(_cameraId, SensorParameter.HdrEnabled, 1);
            }
            else
            {
                status = _camera.SetParameter(_cameraId, SensorParameter.HdrEnabled, 0);
            }
            return status;
        }

        /// <summary>
        /// Reads sensor temperature.
        /// </summary>
        /// <param name="sensorChipTemperature">Sensor chip temperature</param>
        /// <param name="sensorBoardTemperature">Sensor board temperature</param>
        /// <param name="illuminatorTemperature">Illuminator temperature</param>
        /// <param name="frontPanelTemperature">Front panel temperature</param>
        /// <returns>Status code indicating operation outcome.</returns>
        public CameraStatusCode GetTemperature(out float sensorChipTemperature, out float sensorBoardTemperature, out float illuminatorTemperature, out float frontPanelTemperature)
        {
            sensorChipTemperature = sensorBoardTemperature = illuminatorTemperature = frontPanelTemperature = 0;
            CameraStatusCode status;
            if (IsHsCamera)
            {
                status = _camera.GetParameter(_cameraId, SensorParameter.ChipTemperature, out sensorChipTemperature);
                if (status == CameraStatusCode.Ok)
                    status = _camera.GetParameter(_cameraId, SensorParameter.SensorBoardTemperatureFloat, out sensorBoardTemperature);
                if (status == CameraStatusCode.Ok)
                    status = _camera.GetParameter(_cameraId, SensorParameter.IlluminatorTemperature, out illuminatorTemperature);
                if (status == CameraStatusCode.Ok)
                    status = _camera.GetParameter(_cameraId, SensorParameter.FrontPanelTemperature, out frontPanelTemperature);
            }
            else
            {
                status = _camera.GetParameter(_cameraId, SensorParameter.SensorTemperature, out sensorChipTemperature);
                if (status == CameraStatusCode.Ok)
                    status = _camera.GetParameter(_cameraId, SensorParameter.SensorBoardTemperature, out sensorBoardTemperature);
            }
            return status;
        }

        /// <summary>
        /// Writes filter values to sensor.
        /// </summary>
        /// <param name="noiseRemoval">Noise removal.</param>
        /// <param name="averageZ">Average Z in µm.</param>
        /// <param name="averageIntensity">Average intensity in µm.</param>
        /// <param name="medianZ">Median Z in pixels.</param>
        /// <param name="medianIntensity">Median intensity in pixels.</param>
        /// <param name="reSample">Re-sample X-resolution in µm.</param>
        /// <param name="peakXFilter">Peak X-filter length</param>
        /// <param name="fillGapMax">Fill gap max in µm.</param>
        /// <param name="trimEdges">Trim edges on/off.</param>
        /// <param name="clusteringX">Clustering distance X in µm.</param>
        /// <param name="clusteringZ">Clustering distance Z in µm.</param>
        /// <param name="clusterMin">Minimum length for cluster in µm.</param>
        public CameraStatusCode SetFilterValues(int noiseRemoval, double averageZ, double averageIntensity, int medianZ, int medianIntensity, double reSample, 
            int peakXFilter, double fillGapMax, bool trimEdges, double clusteringX, double clusteringZ, double clusterMin)
        {
            var grabbingStatus = StopGrabbing(30);

            var status = _camera.SetParameter(_cameraId, SensorParameter.NoiseRemoval, noiseRemoval);
            if (status == CameraStatusCode.Ok)
                status = _camera.SetParameter(_cameraId, SensorParameter.AverageZFilterSize, averageZ);
            if (status == CameraStatusCode.Ok)
                status = _camera.SetParameter(_cameraId, SensorParameter.AverageIntensityFilterSize, averageIntensity);
            if (status == CameraStatusCode.Ok)
                status = _camera.SetParameter(_cameraId, SensorParameter.MedianZFilterSize, medianZ);
            if (status == CameraStatusCode.Ok)
                status = _camera.SetParameter(_cameraId, SensorParameter.MedianIntensityFilterSize, medianIntensity);
            if (status == CameraStatusCode.Ok)
                status = _camera.SetParameter(_cameraId, SensorParameter.ResampleLineXResolution, reSample);
	        if (status == CameraStatusCode.Ok && IsXFilterSupported)
				status  = _camera.SetParameter(_cameraId, SensorParameter.PeakXFilter, peakXFilter);
            if (status == CameraStatusCode.Ok)
                status = _camera.SetParameter(_cameraId, SensorParameter.FillGapXmax, fillGapMax);
            if (status == CameraStatusCode.Ok)
                status = _camera.SetParameter(_cameraId, SensorParameter.TrimEdges, trimEdges ? 1 : 0);
            if (status == CameraStatusCode.Ok)
                status = _camera.SetParameter(_cameraId, SensorParameter.ClusteringDistanceX, clusteringX);
            if (status == CameraStatusCode.Ok)
                status = _camera.SetParameter(_cameraId, SensorParameter.ClusteringDistanceZ, clusteringZ);
            if (status == CameraStatusCode.Ok)
                status = _camera.SetParameter(_cameraId, SensorParameter.ClusterMinimumLength, clusterMin);

            if (grabbingStatus != NotModified)
                StartGrabbing(20);

            return status;
        }

	    public int FlushQueue(int idleTime)
	    {
		    return (int) _camera.SetParameter(_cameraId, SensorParameter.FlushQueue, idleTime);
	    }

        /// <summary>
        /// Sets filter values for signal detection and peak average intensity filters for camera.
        /// </summary>
        /// <param name="peakDetection">Filter to reduce noise at low peak values</param>
        /// <param name="averageIntensity">Filter for averaging intensity values of the detected peaks</param>
        /// <returns>Status code indicating operation outcome.</returns>
        public CameraStatusCode SetSignalDetectionFilterLength(int peakDetection, int averageIntensity)
        {
            CameraStatusCode status = _camera.SetParameter(_cameraId, SensorParameter.SignalDetectionFilterLength, peakDetection);
            if (status == CameraStatusCode.Ok)
                status = _camera.SetParameter(_cameraId, SensorParameter.PeakAverageIntensityFilterLength, averageIntensity);
            return status;
        }

        /// <summary>
        /// Sets layer specific settings to camera.
        /// </summary>
        /// <returns>Status code indicating operation outcome.</returns>
        public CameraStatusCode SetLayerSpecificSettings()
        {
            CameraStatusCode status = CameraStatusCode.Ok;
            for (int layer = 0; layer < _parameters.Layers.Length; layer++)
            {
                if (status != CameraStatusCode.Ok) break;
                status = _camera.SetLayerParameter(_cameraId, layer, SensorParameter.LayerMaxThickness, (float)_parameters.Layers[layer].MaxThickness);
                if (status != CameraStatusCode.Ok) break;
                status = _camera.SetLayerParameter(_cameraId, layer, SensorParameter.LayerIntensityType, _parameters.Layers[layer].IntensityType);
            }

            return status;
        }

        /// <summary>
        /// Sets refractive indexes to camera.
        /// </summary>
        /// <returns>Status code indicating operation outcome.</returns>
        public CameraStatusCode SetRefractiveIndexes()
        {
            CameraStatusCode status = CameraStatusCode.Ok;
            for (int layer = 0; layer < _parameters.Layers.Length; ++layer)
            { 
                status = _camera.SetLayerParameter(_cameraId, layer, SensorParameter.LayerEffectiveRefractiveIndex, (float)_parameters.Layers[layer].RefractiveIndex);
                if (status != CameraStatusCode.Ok) break;
            }

            return status;
        }

        /// <summary>
        /// Sets thickness mode to camera.
        /// </summary>
        /// <param name="thickness">Thickness mode on off</param>
        /// <returns>Status code indicating operation outcome.</returns>
        public CameraStatusCode SetThicknessMode(bool thickness)
        {
            return _camera.SetParameter(_cameraId, SensorParameter.ThicknessMode, thickness ? 1 : 0);
        }

        /// <summary>
        /// Sets layer minimum thickness to camera.
        /// </summary>
        /// <param name="thickness">Minimum thickness</param>
        /// <returns>Status code indicating operation outcome.</returns>
        public CameraStatusCode SetMinimumThickness(float thickness)
        {
            CameraStatusCode status = CameraStatusCode.Ok;
            status = _camera.SetParameter(_cameraId, SensorParameter.LayerMinThickness, thickness);
            for (int layer = 0; layer < _parameters.Layers.Length; ++layer)
            {
                // Currently writes the same value for each layer
                status = _camera.SetLayerParameter(_cameraId, layer, SensorParameter.LayerMinThickness, thickness);
            }

            return status;
        }

        /// <summary>
        /// Sets peak detection parameters to camera.
        /// </summary>
        /// <param name="materialType">Material type</param>
        /// <param name="sensitivity">Detection sensitivity</param>
        /// <returns>Status code indicating operation outcome.</returns>
        public CameraStatusCode SetPeakDetectionParameters(MaterialType materialType, DetectionSensitivity sensitivity)
        {
            return _camera.SetPeakDetectionParameters(_cameraId, materialType, sensitivity);
        }

        /// <summary>
        /// Load camera parameters recipe
        /// </summary>
        /// <param name="recipe">Recipe file name</param>
        /// <returns>Status code indicating operation outcome.</returns>
        public CameraStatusCode LoadRecipe(string recipe)
        {
            RecipeFileName = recipe;
            CameraStatusCode status = _camera.SetParameter(_cameraId, SensorParameter.LoadRecipe, RecipeFileName);
            ReloadSensorParameters();

            return status;
        }

        /// <summary>
        /// Save camera parameters recipe
        /// </summary>
        /// <param name="recipe">Recipe file name</param>
        /// <returns>Status code indicating operation outcome.</returns>
        public CameraStatusCode SaveRecipe(string recipe)
        {
            RecipeFileName = recipe;
            return _camera.SetParameter(_cameraId, SensorParameter.SaveRecipe, RecipeFileName);
        }

        public CameraStatusCode ReloadSensorParameters()
        {
            CameraStatusCode status = _camera.GetParameter(_cameraId, SensorParameter.AgcEnabled, out int agcEnabled);
            if (status == CameraStatusCode.Ok)
            {
                _parameters.IsAgcEnabled = agcEnabled == 1;
                IsAgcEnabled = _parameters.IsAgcEnabled;
            }
            status = _camera.GetParameter(_cameraId, SensorParameter.AgcTarget, out float targetIntensity);
            if (status == CameraStatusCode.Ok)
                _parameters.AgcTargetIntensity = targetIntensity;
            
            _parameters.LedPulseWidth = CurrentLedPulseWidth;
            status = _camera.GetParameter(_cameraId, SensorParameter.AgcWiLimit, out int maxLedPulse);
            if (status == CameraStatusCode.Ok)
                _parameters.MaxLedPulseWidth = maxLedPulse;
            status = _camera.GetParameter(_cameraId, SensorParameter.PeakEnabled, out int isPeakEnabled);
            if (status == CameraStatusCode.Ok)
                _parameters.IsPeakEnabled = isPeakEnabled != 0;
            status = _camera.GetParameter(_cameraId, SensorParameter.PulseFrequency, out int frequency);
            if (status == CameraStatusCode.Ok && frequency > 0 && _parameters.IsPeakEnabled)
                _parameters.Freq = frequency;
            _parameters.IsExternalPulsingEnabled = frequency == 0;
            status = _camera.GetParameter(_cameraId, SensorParameter.PulseDivider, out int divider);
            if (status == CameraStatusCode.Ok) 
                _parameters.PulseDivider = divider;
            status = _camera.GetParameter(_cameraId, SensorParameter.Mtu, out int jumbo);
            if (status == CameraStatusCode.Ok) 
                _parameters.JumboFrameMtu = jumbo;
            status = _camera.GetParameter(_cameraId, SensorParameter.PulseCurrentMiddle, out double current);
            if (status == CameraStatusCode.Ok)
                _parameters.Current = current;
            status = _camera.GetParameter(_cameraId, SensorParameter.Gain, out double gain);
            if (status == CameraStatusCode.Ok)
            {
                if (IsHsCamera)
                    _parameters.GainHs = gain;
                else
                    _parameters.Gain = gain;
            }
            status = _camera.GetParameter(_cameraId, SensorParameter.TriggerSource, out int triggerSource);
            if (status == CameraStatusCode.Ok) 
                _parameters.TriggerSource = triggerSource;
            _parameters.FirLength = GetFirLength();
            _parameters.AverFirLength = GetAveragingFirLength();
            int detection = -1, averageIntensity = -1;
            GetSignalDetectionFilterLength(ref detection, ref averageIntensity);
            if (detection >= 0)
                _parameters.DetectionFilter = detection;
            if (averageIntensity >= 0)
                _parameters.AverageIntensityFilter = averageIntensity;
            _parameters.Threshold = GetThreshold();
            status = _camera.GetParameter(_cameraId, SensorParameter.Width, out int width);
            if (status == CameraStatusCode.Ok) 
                _parameters.SensorWidth = width;
            status = _camera.GetParameter(_cameraId, SensorParameter.MaxPointCount, out int maxPointCount);
            if (status == CameraStatusCode.Ok) 
                _parameters.MaxPointCount = maxPointCount;
            status = _camera.GetParameter(_cameraId, SensorParameter.LayerMinThickness, out float thickness);
            if (status == CameraStatusCode.Ok) 
                _parameters.LayerMinThickness = thickness;
            if (IsHdrSupported)
            {
                status = _camera.GetParameter(_cameraId, SensorParameter.HdrEnabled, out int hdr);
                if (status == CameraStatusCode.Ok)
                    _parameters.HdrEnabled = hdr == 1;
                status = _camera.GetParameter(_cameraId, SensorParameter.VLow2Float, out float vLow2); 
                if (status == CameraStatusCode.Ok) 
                    _parameters.HdrVLow2 = vLow2;  
                status = _camera.GetParameter(_cameraId, SensorParameter.VLow3Float, out float vLow3);   
                if (status == CameraStatusCode.Ok)  
                    _parameters.HdrVLow3 = vLow3;
                status = _camera.GetParameter(_cameraId, SensorParameter.Kp1PosFloat, out float kp1Pos);
                if (status == CameraStatusCode.Ok) 
                    _parameters.HdrKp1Pos = kp1Pos;
                status = _camera.GetParameter(_cameraId, SensorParameter.Kp2PosFloat, out float kp2Pos);
                if (status == CameraStatusCode.Ok) 
                    _parameters.HdrKp2Pos = kp2Pos;
            }
            status = _camera.GetParameter(_cameraId, SensorParameter.NoiseRemoval, out int noiseRemoval);
            if (status == CameraStatusCode.Ok)
                _parameters.NoiseRemoval = noiseRemoval;
            status = _camera.GetParameter(_cameraId, SensorParameter.AverageZFilterSize, out double averageZ);
            if (status == CameraStatusCode.Ok)
                _parameters.AverageZFilterSize = averageZ;
            status = _camera.GetParameter(_cameraId, SensorParameter.AverageIntensityFilterSize, out double averageIntensityFilterSize);
            if (status == CameraStatusCode.Ok)
                _parameters.AverageIntensityFilterSize = averageIntensityFilterSize;
            status = _camera.GetParameter(_cameraId, SensorParameter.MedianZFilterSize, out int medianZ);
            if (status == CameraStatusCode.Ok)
                _parameters.MedianZFilterSize = medianZ;
            status = _camera.GetParameter(_cameraId, SensorParameter.MedianIntensityFilterSize, out int medianIntensity);
            if (status == CameraStatusCode.Ok)
                _parameters.MedianIntensityFilterSize = medianIntensity;
            status = _camera.GetParameter(_cameraId, SensorParameter.ResampleLineXResolution, out double reSample);
            if (status == CameraStatusCode.Ok)
                _parameters.ResampleLineXResolution = reSample;
            if (IsXFilterSupported)
            {
                status = _camera.GetParameter(_cameraId, SensorParameter.PeakXFilter, out int peakXFilter);
                if (status == CameraStatusCode.Ok)
                    _parameters.PeakXFilter = peakXFilter;
            }
            //status = _camera.GetParameter(_cameraId, SensorParameter.ThicknessMode, out int thicknessMode);
            //if (status == CameraStatusCode.Ok)
            //    _parameters.IsThicknessMode = thicknessMode == 1;
            status = _camera.GetParameter(_cameraId, SensorParameter.FillGapXmax, out float fillGapMax);
            if (status == CameraStatusCode.Ok)
                _parameters.FillGapMax = fillGapMax;
            status = _camera.GetParameter(_cameraId, SensorParameter.TrimEdges, out int trimEdges);
            if (status == CameraStatusCode.Ok)
                _parameters.IsTrimEdges = trimEdges == 1;
            status = _camera.GetParameter(_cameraId, SensorParameter.ClusteringDistanceX, out float clusteringX);
            if (status == CameraStatusCode.Ok)
                _parameters.ClusteringX = clusteringX;
            status = _camera.GetParameter(_cameraId, SensorParameter.ClusteringDistanceZ, out float clusteringZ);
            if (status == CameraStatusCode.Ok)
                _parameters.ClusteringZ = clusteringZ;
            status = _camera.GetParameter(_cameraId, SensorParameter.ClusterMinimumLength, out float clusterMin);
            if (status == CameraStatusCode.Ok)
                _parameters.ClusterMin = clusterMin;
            for (int layer = 0; layer < _parameters.Layers.Length; layer++)
            {
                status = _camera.GetLayerParameter(_cameraId, layer, SensorParameter.LayerEffectiveRefractiveIndex, out float refractiveIndex);
                if (status == CameraStatusCode.Ok)
                    _parameters.Layers[layer].RefractiveIndex = refractiveIndex;
            } 
            for (int layer = 0; layer < _parameters.Layers.Length; layer++)
            {
                status = _camera.GetLayerParameter(_cameraId, layer, SensorParameter.LayerMaxThickness, out float maxThickness);
                if (status == CameraStatusCode.Ok)
                    _parameters.Layers[layer].MaxThickness = maxThickness;
                status = _camera.GetLayerParameter(_cameraId, layer, SensorParameter.LayerMinThickness, out float minThickness);
                if (status == CameraStatusCode.Ok)
                    _parameters.Layers[layer].MinThickness = minThickness;
                status = _camera.GetLayerParameter(_cameraId, layer, SensorParameter.LayerIntensityType, out int intensityType);
                if (status == CameraStatusCode.Ok)
                    _parameters.Layers[layer].IntensityType = intensityType;
            }

            return status;
        }

        public int GetFirLength()
        {
            if (_camera.GetParameter(_cameraId, SensorParameter.FirLength, out int firLength) != CameraStatusCode.Ok)
                return 16;
            return firLength;
        }

        public int GetAveragingFirLength()
        {
            if (_camera.GetParameter(_cameraId, SensorParameter.FirAverLength, out int averagingFirLength) != CameraStatusCode.Ok)
                return 16;
            return averagingFirLength;
        }

        public void GetSignalDetectionFilterLength(ref int detection, ref int averageIntensity)
        {
            if (_camera.GetParameter(_cameraId, SensorParameter.SignalDetectionFilterLength, out int detectionRead) == CameraStatusCode.Ok)
                detection = detectionRead;
            if (_camera.GetParameter(_cameraId, SensorParameter.PeakAverageIntensityFilterLength, out int averageIntensityRead) == CameraStatusCode.Ok)
                averageIntensity = averageIntensityRead;
        }

        public int GetThreshold()
        {
            if (_camera.GetParameter(_cameraId, SensorParameter.PeakThreshold, out int threshold) != CameraStatusCode.Ok)
                return 17;
            return threshold;
        }

        /// <summary>
        /// Checks if connection to connected sensor is still ok.
        /// </summary>
        /// <returns>True is connection is ok.</returns>
        public bool IsConnected()
        {
            // perform continues check status only if connection has successfully been created   
            if (!ConnectionStatus) return true;
            var status = _camera.IsConnected(_cameraId);
            if (status == CameraStatusCode.NotConnected)
            {
                ConnectionStatus = false;
            }
            return (status == CameraStatusCode.Ok);
        }
    }
}
