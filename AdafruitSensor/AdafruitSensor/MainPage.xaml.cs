using System;
using System.Diagnostics;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Runtime.InteropServices.WindowsRuntime;
using Windows.Foundation;
using Windows.Foundation.Collections;
using Windows.UI.Xaml;
using Windows.UI.Xaml.Controls;
using Windows.UI.Xaml.Controls.Primitives;
using Windows.UI.Xaml.Data;
using Windows.UI.Xaml.Input;
using Windows.UI.Xaml.Media;
using Windows.UI.Xaml.Navigation;
using Windows.Devices.Enumeration;
using Windows.Devices.I2c;
using System.Threading.Tasks;
using System.Runtime.Serialization.Json;
using System.Text;



// The Blank Page item template is documented at http://go.microsoft.com/fwlink/?LinkId=402352&clcid=0x409

namespace AdafruitSensor
{
    /// <summary>
    /// An empty page that can be used on its own or navigated to within a Frame.
    /// </summary>
    public sealed partial class MainPage : Page
    {

        // I2C Device
        private I2cDevice I2CDevbmp;
        private I2cDevice I2CDevgyro;
        private I2cDevice I2CDevaccel;
        private I2cDevice I2CDevmag;

        Adafruit_BMP085_Unified bmp;
        Adafruit_L3GD20_Unified gyro;
        Adafruit_LSM303_Accel_Unified accel;
        Adafruit_LSM303_Mag_Unified mag;

        private Adafruit_BMP085_Unified.sensor_t bmpSensor;
        private Adafruit_L3GD20_Unified.sensor_t gyroSensor;
        private Adafruit_LSM303_Accel_Unified.sensor_t accelSensor;
        private Adafruit_LSM303_Mag_Unified.sensor_t magSensor;

        private Adafruit_BMP085_Unified.sensors_event_t Pressevent;
        private Adafruit_L3GD20_Unified.sensors_event_t Gyroevent;
        private Adafruit_LSM303_Accel_Unified.sensors_event_t Accelevent;
        private Adafruit_LSM303_Mag_Unified.sensors_event_t Magevent;

        public static ServerTask st = new ServerTask();

        

        // Timer
        private DispatcherTimer ReadSensorTimer;
        
                    
        public MainPage()
        {
            this.InitializeComponent();
            InitializeAdF_BMP085_U_I2CDevice();
            InitializeAdF_L3GD20_U_I2CDevice();
            InitializeAdF_LSM303_Accel_I2CDevice();
            InitializeAdF_LSM303_Mag_I2CDevice();
          
            st.InitEventHubConnection();
            Task.Delay(300).Wait();
          
        }

        private void MainPage_Unloaded(object sender, object args)
        {
            // Cleanup
            I2CDevbmp.Dispose();
            I2CDevgyro.Dispose();
            I2CDevmag.Dispose();
            I2CDevaccel.Dispose();
            
            

        }

        private async void InitializeAdF_LSM303_Mag_I2CDevice()
        {
            try
            {
                // Initialize I2C device
                var settings = new I2cConnectionSettings(Adafruit_LSM303_Mag_Unified.LSM303_ADDRESS_MAG);

                settings.BusSpeed = I2cBusSpeed.FastMode;
                settings.SharingMode = I2cSharingMode.Shared;

                string aqs = I2cDevice.GetDeviceSelector("I2C1");  /* Find the selector string for the I2C bus controller                   */
                var dis = await DeviceInformation.FindAllAsync(aqs);            /* Find the I2C bus controller device with our selector string           */

                I2CDevmag = await I2cDevice.FromIdAsync(dis[0].Id, settings);    /* Create an I2cDevice with our selected bus controller and I2C settings */

                mag = new Adafruit_LSM303_Mag_Unified(ref I2CDevmag, 30302);
                magSensor = mag.getSensorObj();
                mag.getSensor(ref magSensor);
                Debug.WriteLine("------------- MAGNETOMETER -----------");
                Debug.WriteLine("Sensor:       " + magSensor.name);
                Debug.WriteLine("Driver Ver:   " + magSensor.version);
                Debug.WriteLine("Unique ID:    " + magSensor.sensor_id);
                Debug.WriteLine("Max Value:    " + magSensor.max_value + " uT");
                Debug.WriteLine("Min Value:    " + magSensor.min_value + " uT");
                Debug.WriteLine("Resolution:   " + magSensor.resolution + " uT");


            }
            catch (Exception e)
            {
                Debug.WriteLine(e.ToString());

                return;
            }


        }
        private async void InitializeAdF_LSM303_Accel_I2CDevice()
        {
            try
            {
                // Initialize I2C device
                var settings = new I2cConnectionSettings(Adafruit_LSM303_Accel_Unified.LSM303_ADDRESS_ACCEL);

                settings.BusSpeed = I2cBusSpeed.FastMode;
                settings.SharingMode = I2cSharingMode.Shared;

                string aqs = I2cDevice.GetDeviceSelector("I2C1");  /* Find the selector string for the I2C bus controller                   */
                var dis = await DeviceInformation.FindAllAsync(aqs);            /* Find the I2C bus controller device with our selector string           */

                I2CDevaccel = await I2cDevice.FromIdAsync(dis[0].Id, settings);    /* Create an I2cDevice with our selected bus controller and I2C settings */

                accel = new Adafruit_LSM303_Accel_Unified(ref I2CDevaccel, 30301);
                accelSensor = accel.getSensorObj();
                accel.getSensor(ref accelSensor);
                Debug.WriteLine("------------- ACCELEROMETER -----------");
                Debug.WriteLine("Sensor:       " + accelSensor.name);
                Debug.WriteLine("Driver Ver:   " + accelSensor.version);
                Debug.WriteLine("Unique ID:    " + accelSensor.sensor_id);
                Debug.WriteLine("Max Value:    " + accelSensor.max_value + " m/s^2");
                Debug.WriteLine("Min Value:    " + accelSensor.min_value + " m/s^2");
                Debug.WriteLine("Resolution:   " + accelSensor.resolution + " m/s^2");


            }
            catch (Exception e)
            {
                Debug.WriteLine(e.ToString());

                return;
            }


        }

        public T Deserialize<T>(string json)
        {
            var _Bytes = Encoding.Unicode.GetBytes(json);
            using (MemoryStream _Stream = new MemoryStream(_Bytes))
            {
                var _Serializer = new DataContractJsonSerializer(typeof(T));
                return (T)_Serializer.ReadObject(_Stream);
            }
        }

        public string Serialize(object instance)
        {
            try
            {
                using (MemoryStream _Stream = new MemoryStream())
                {
                    var _Serializer = new DataContractJsonSerializer(instance.GetType());
                    _Serializer.WriteObject(_Stream, instance);
                    _Stream.Position = 0;
                    using (StreamReader _Reader = new StreamReader(_Stream))
                    { return _Reader.ReadToEnd(); }
                }
            }
            catch (Exception e)
            {
                Debug.WriteLine(e.ToString());
                return null;

            }

        }

        public void EventHubSend(String Data)
        {
            try
            {

               st.sendMessage(Data);

            }
            catch (Exception e)
            {
                Debug.WriteLine(e.ToString());

            }
        }

        private async void InitializeAdF_L3GD20_U_I2CDevice()
        {
            try
            {
                // Initialize I2C device
                var settings = new I2cConnectionSettings(Adafruit_L3GD20_Unified.L3GD20_ADDRESS);

                settings.BusSpeed = I2cBusSpeed.FastMode;
                settings.SharingMode = I2cSharingMode.Shared;

                string aqs = I2cDevice.GetDeviceSelector("I2C1");  /* Find the selector string for the I2C bus controller                   */
                var dis = await DeviceInformation.FindAllAsync(aqs);            /* Find the I2C bus controller device with our selector string           */

                I2CDevgyro = await I2cDevice.FromIdAsync(dis[0].Id, settings);    /* Create an I2cDevice with our selected bus controller and I2C settings */

                gyro = new Adafruit_L3GD20_Unified(ref I2CDevgyro,20);
                gyroSensor = gyro.getSensorObj();
                gyro.getSensor(ref gyroSensor);
                Debug.WriteLine("------------- GYROSCOPE -----------");
                Debug.WriteLine("Sensor:       " + gyroSensor.name);
                Debug.WriteLine("Driver Ver:   " + gyroSensor.version);
                Debug.WriteLine("Unique ID:    " + gyroSensor.sensor_id);
                Debug.WriteLine("Max Value:    " + gyroSensor.max_value + " rad/s");
                Debug.WriteLine("Min Value:    " + gyroSensor.min_value + " rad/s");
                Debug.WriteLine("Resolution:   " + gyroSensor.resolution + " rad/s");

                
            }
            catch (Exception e)
            {
                Debug.WriteLine(e.ToString());

                return;
            }

           
        }
       


    private async void InitializeAdF_BMP085_U_I2CDevice()
        {
            try
            {
                // Initialize I2C device
                var settings = new I2cConnectionSettings(Adafruit_BMP085_Unified.BMP085_ADDRESS);

                settings.BusSpeed = I2cBusSpeed.FastMode;
                settings.SharingMode = I2cSharingMode.Shared;

                string aqs = I2cDevice.GetDeviceSelector("I2C1");  /* Find the selector string for the I2C bus controller                   */
                var dis = await DeviceInformation.FindAllAsync(aqs);            /* Find the I2C bus controller device with our selector string           */

                I2CDevbmp = await I2cDevice.FromIdAsync(dis[0].Id, settings);    /* Create an I2cDevice with our selected bus controller and I2C settings */

                bmp = new Adafruit_BMP085_Unified(ref I2CDevbmp, 18001);
                bmpSensor = bmp.getSensorObj();
                bmp.getSensor(ref bmpSensor);
                Debug.WriteLine("-------- PRESSURE/ALTITUDE ---------");
                Debug.WriteLine("Sensor:       " + bmpSensor.name);
                Debug.WriteLine("Driver Ver:   " + bmpSensor.version);
                Debug.WriteLine("Unique ID:    " + bmpSensor.sensor_id);
                Debug.WriteLine("Max Value:    " + bmpSensor.max_value + " hPa");
                Debug.WriteLine("Min Value:    " + bmpSensor.min_value + " hpa");
                Debug.WriteLine("Resolution:   " + bmpSensor.resolution + " hPa");


            }
            catch (Exception e)
            {
                Debug.WriteLine(e.ToString());

                return;
            }


        }
        private  void  Timer_Tick(object sender, object e)
        {

            /* Display the results (acceleration is measured in m/s^2) */
            accel.getEvent(ref Accelevent);
            Debug.WriteLine("ACCEL  ");
            Debug.WriteLine("X: " + Accelevent.acceleration.x + "  " + "Y: " + Accelevent.acceleration.y + "  " + "Z: " + Accelevent.acceleration.z + "  m/s^2");
            AccelXaxis.Text = Accelevent.acceleration.x.ToString();
            AccelYaxis.Text = Accelevent.acceleration.y.ToString();
            AccelZaxis.Text = Accelevent.acceleration.z.ToString();
            /* Display the results (magnetic vector values are in micro-Tesla (uT)) */
            mag.getEvent(ref Magevent);
            Debug.WriteLine("MAG  ");
            Debug.WriteLine("X: " + Magevent.magnetic.x + "  " + "Y: " + Magevent.magnetic.y + "  " + "Z: " + Magevent.magnetic.z + "  uT");
            MagXaxis.Text = Magevent.magnetic.x.ToString();
            MagYaxis.Text = Magevent.magnetic.y.ToString();
            MagZaxis.Text = Magevent.magnetic.z.ToString();
            /* Display the results (gyrocope values in rad/s) */
            gyro.getEvent(ref Gyroevent);
            Debug.WriteLine("GYRO  ");
            Debug.WriteLine("X: " + Gyroevent.gyro.x + "  " + "Y: " + Gyroevent.gyro.y + "  " + "Z: " + Gyroevent.gyro.z + "  rad/s");
            GyroXaxis.Text = Gyroevent.gyro.x.ToString();
            GyroYaxis.Text = Gyroevent.gyro.y.ToString();
            GyroZaxis.Text = Gyroevent.gyro.z.ToString();
            /* Display the pressure sensor results (barometric pressure is measure in hPa) */
            bmp.getEvent(ref Pressevent);
            if (Pressevent.pressure != 0)
            {
                /* Display atmospheric pressure in hPa */
                Debug.WriteLine("PRESS :" + Pressevent.pressure + " hPa ");
                Pressure.Text = Pressevent.pressure.ToString();
                /* Display ambient temperature in C */
                float temperature = 0;
                bmp.getTemperature(ref temperature);
                Debug.WriteLine("TEMP :"+ temperature + " C ");
                Temperature.Text = temperature.ToString();
                /* Then convert the atmospheric pressure, SLP and temp to altitude    */
                /* Update this next line with the current SLP for better results      */
                float seaLevelPressure = Adafruit_BMP085_Unified.SENSORS_PRESSURE_SEALEVELHPA;
                float altitude = bmp.pressureToAltitude(seaLevelPressure, Pressevent.pressure, temperature);
                Debug.WriteLine("ALTI :" + altitude + " m " );
                Altitude.Text = altitude.ToString();
                DateTime.Text = System.DateTime.Now.ToString();
                SensorData SensorDataObj = new SensorData(Accelevent.acceleration.x, Accelevent.acceleration.y, Accelevent.acceleration.z,
                    Magevent.magnetic.x, Magevent.magnetic.y, Magevent.magnetic.z, Gyroevent.gyro.x, Gyroevent.gyro.y, Gyroevent.gyro.z,
                    Pressevent.pressure, temperature, altitude, System.DateTime.Now.ToString("dd-MMM-yyyy HH:mm:ss tt"));
                String JsonData = Serialize(SensorDataObj);
                 EventHubSend(JsonData);

            }

        }
        private void Start_Click(object sender, RoutedEventArgs e)
        {
            Start.IsEnabled = false;
            
            if (!bmp.begin())
            {
                /* There was a problem detecting the BMP085 ... check your connections */
                Debug.WriteLine("Ooops, no BMP085 detected ... Check your wiring or I2C ADDR!");
                while (true) ;
            }
            Pressevent = bmp.getSensorEventObj();

            if (!gyro.begin())
            {
                /* There was a problem detecting the BMP085 ... check your connections */
                Debug.WriteLine("Ooops, no L3GD20 detected ... Check your wiring or I2C ADDR!");
                while (true) ;
            }
            Gyroevent = gyro.getSensorEventObj();

            if (!accel.begin())
            {
                /* There was a problem detecting the BMP085 ... check your connections */
                Debug.WriteLine("Ooops, no LSM303 detected ... Check your wiring or I2C ADDR!");
                while (true) ;
            }
            Accelevent = accel.getSensorEventObj();

            if (!mag.begin())
            {
                /* There was a problem detecting the BMP085 ... check your connections */
                Debug.WriteLine("Ooops, no LSM303 detected ... Check your wiring or I2C ADDR!");
                while (true) ;
            }
            Magevent = mag.getSensorEventObj();

            // Start Timer every 1 seconds
            ReadSensorTimer = new DispatcherTimer();
            ReadSensorTimer.Interval = TimeSpan.FromMilliseconds(1000);
            ReadSensorTimer.Tick += Timer_Tick;
            ReadSensorTimer.Start();
        }

        private void Exit_Click(object sender, RoutedEventArgs e)
        {
            Application.Current.Exit();
        }
    }

    public class SensorData
    {
        public float accel_x;
        public float accel_y;
        public float accel_z;
        public float mag_x;
        public float mag_y;
        public float mag_z;
        public float gyro_x;
        public float gyro_y;
        public float gyro_z;
        public float pressure;
        public float temperature;
        public float altitude;
        public string Currentdatetime;
        public string DeviceName;

        public SensorData() { } //required for JSON serializers
        public SensorData(float acc_x, float acc_y, float acc_z,
                          float ma_x, float ma_y, float ma_z,
                          float gyr_x, float gyr_y, float gyr_z,
                          float press, float temp, float alt, string Cdatetime)
        {
            accel_x = acc_x;
            accel_y = acc_y;
            accel_z = acc_z;
            mag_x = ma_x;
            mag_y = ma_y;
            mag_z = ma_z;
            gyro_x = gyr_x;
            gyro_y = gyr_y;
            gyro_z = gyr_z;
            pressure = press;
            temperature = temp;
            altitude = alt;
            Currentdatetime = Cdatetime;
            DeviceName = "Raspberry Pi-2";
        }
        
    }

}
