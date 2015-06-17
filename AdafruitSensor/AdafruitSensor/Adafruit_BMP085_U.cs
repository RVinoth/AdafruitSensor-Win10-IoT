using System;
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
using System.Threading.Tasks;
using Windows.Devices.Enumeration;
using Windows.Devices.I2c;


namespace AdafruitSensor
{
    class Adafruit_BMP085_Unified
    {

        
        // I2C Device
        private I2cDevice I2CDev;

        /*=========================================================================
            I2C ADDRESS/BITS
            -----------------------------------------------------------------------*/
        public const int BMP085_ADDRESS = (0x77);
        /*=========================================================================*/
        //< Average sea level pressure is 1013.25 hPa
        public const float SENSORS_PRESSURE_SEALEVELHPA = (1013.25F);
        /*=========================================================================
            REGISTERS
            -----------------------------------------------------------------------*/

        private const byte BMP085_REGISTER_CAL_AC1 = 0xAA;  // R   Calibration data (16 bits)
        private const byte BMP085_REGISTER_CAL_AC2 = 0xAC;  // R   Calibration data (16 bits)
        private const byte BMP085_REGISTER_CAL_AC3 = 0xAE;  // R   Calibration data (16 bits)
        private const byte BMP085_REGISTER_CAL_AC4 = 0xB0;  // R   Calibration data (16 bits)
        private const byte BMP085_REGISTER_CAL_AC5 = 0xB2;  // R   Calibration data (16 bits)
        private const byte BMP085_REGISTER_CAL_AC6 = 0xB4;  // R   Calibration data (16 bits)
        private const byte BMP085_REGISTER_CAL_B1 = 0xB6; // R   Calibration data (16 bits)
        private const byte BMP085_REGISTER_CAL_B2 = 0xB8;  // R   Calibration data (16 bits)
        private const byte BMP085_REGISTER_CAL_MB = 0xBA;  // R   Calibration data (16 bits)
        private const byte BMP085_REGISTER_CAL_MC = 0xBC;  // R   Calibration data (16 bits)
        private const byte BMP085_REGISTER_CAL_MD = 0xBE;  // R   Calibration data (16 bits)
        private const byte BMP085_REGISTER_CHIPID = 0xD0;
        private const byte BMP085_REGISTER_VERSION = 0xD1;
        private const byte BMP085_REGISTER_SOFTRESET = 0xE0;
        private const byte BMP085_REGISTER_CONTROL = 0xF4;
        private const byte BMP085_REGISTER_TEMPDATA = 0xF6;
        private const byte BMP085_REGISTER_PRESSUREDATA = 0xF6;
        private const byte BMP085_REGISTER_READTEMPCMD = 0x2E;
        private const byte BMP085_REGISTER_READPRESSURECMD = 0x34;        
        /*=========================================================================*/

        /*=========================================================================
            MODE SETTINGS
            -----------------------------------------------------------------------*/
       public enum bmp085_mode_t
        {
            BMP085_MODE_ULTRALOWPOWER = 0,
            BMP085_MODE_STANDARD = 1,
            BMP085_MODE_HIGHRES = 2,
            BMP085_MODE_ULTRAHIGHRES = 3
        };
/*=========================================================================*/

/*=========================================================================
    CALIBRATION DATA
    -----------------------------------------------------------------------*/
    public struct bmp085_calib_data
        {
            public short ac1;
            public short ac2;
            public short ac3;
            public ushort ac4;
            public ushort ac5;
            public ushort ac6;
            public short b1;
            public short b2;
            public short mb;
            public short mc;
            public short md;
    };
        /*=========================================================================*/
    private bmp085_calib_data _bmp085_coeffs;   // Last read accelerometer data will be available here
    private byte _bmp085Mode;
    private int _sensorID;

        /* Sensor event (36 bytes) */
        /** struct sensor_event_s is used to provide a single sensor event in a common format. */
        public struct sensors_event_t
        {
           public int version;                          /**< must be sizeof(struct sensors_event_t) */
           public int sensor_id;                        /**< unique sensor identifier */
           public int type;                             /**< sensor type */
           public int reserved0;                        /**< reserved */
           public int timestamp;                        /**< time is in milliseconds */
           public float pressure;             /**< pressure in hectopascal (hPa) */
         };

        private sensors_event_t SensEvent;

        /* Sensor details (40 bytes) */
        /** struct sensor_s is used to describe basic information about a specific sensor. */
        public struct sensor_t
        {
            public string name;                        /**< sensor name */
            public int version;                         /**< version of the hardware + driver */
            public int sensor_id;                       /**< unique sensor identifier */
            public int type;                            /**< this sensor's type (ex. SENSOR_TYPE_LIGHT) */
            public float max_value;                       /**< maximum value of this sensor's value in SI units */
            public float min_value;                       /**< minimum value of this sensor's value in SI units */
            public float resolution;                      /**< smallest difference between two values reported by this sensor */
            public int min_delay;                       /**< min delay in microseconds between events. zero = not a constant rate */
        };

        private sensor_t Sensor;
        /***************************************************************************
         PRIVATE FUNCTIONS
         ***************************************************************************/

        /**************************************************************************/
        /*!
            @brief  Writes an 8 bit value over I2C
        */
        /**************************************************************************/
         private void writeCommand(byte reg, byte value)
        {
            I2CDev.Write(new byte[] { reg , value });
        }

        /**************************************************************************/
        /*!
            @brief  Reads an 8 bit value over I2C
        */
        /**************************************************************************/
         private void read8(byte reg, ref byte value)
        {
            byte[] addr = new byte[] { reg };
            byte[] data = new byte[1];
            I2CDev.WriteRead(addr, data);
            value = data[0];
        }

        /**************************************************************************/
        /*!
            @brief  Reads a 16 bit value over I2C
        */
        /**************************************************************************/
        private void read16(byte reg, ref ushort value)
        {
            byte[] addr = new byte[] { reg };
            byte[] data = new byte[2];
            I2CDev.WriteRead(addr, data);
            value = (ushort)((data[0] << 8) | (data[1])); 
        }

        /**************************************************************************/
        /*!
            @brief  Reads a signed 16 bit value over I2C
        */
        /**************************************************************************/
         private void readS16(byte reg, ref short value)
        {
            ushort i=0;
            read16(reg, ref i);
            value = (short)i;
        }

        /**************************************************************************/
        /*!
            @brief  Reads the factory-set coefficients
        */
        /**************************************************************************/
         private void readCoefficients()
        {

            readS16(BMP085_REGISTER_CAL_AC1, ref _bmp085_coeffs.ac1);
            readS16(BMP085_REGISTER_CAL_AC2, ref _bmp085_coeffs.ac2);
            readS16(BMP085_REGISTER_CAL_AC3, ref _bmp085_coeffs.ac3);
            read16(BMP085_REGISTER_CAL_AC4, ref _bmp085_coeffs.ac4);
            read16(BMP085_REGISTER_CAL_AC5, ref _bmp085_coeffs.ac5);
            read16(BMP085_REGISTER_CAL_AC6, ref _bmp085_coeffs.ac6);
            readS16(BMP085_REGISTER_CAL_B1, ref _bmp085_coeffs.b1);
            readS16(BMP085_REGISTER_CAL_B2, ref _bmp085_coeffs.b2);
            readS16(BMP085_REGISTER_CAL_MB, ref _bmp085_coeffs.mb);
            readS16(BMP085_REGISTER_CAL_MC, ref _bmp085_coeffs.mc);
            readS16(BMP085_REGISTER_CAL_MD, ref _bmp085_coeffs.md);

        }

        /**************************************************************************/
        /*!

        */
        /**************************************************************************/
        private void readRawTemperature(ref int temperature)
        {

            ushort t=0;
            writeCommand(BMP085_REGISTER_CONTROL, BMP085_REGISTER_READTEMPCMD);
             Task.Delay(5).Wait();
            read16(BMP085_REGISTER_TEMPDATA, ref t);
            temperature = t;

        }

        /**************************************************************************/
        /*!

        */
        /**************************************************************************/
        private void readRawPressure(ref int pressure)
        {

            byte  p8=0;
            ushort p16=0;
            int  p32=0;
            
            writeCommand(BMP085_REGISTER_CONTROL, (byte)(BMP085_REGISTER_READPRESSURECMD + (_bmp085Mode << (byte)6)));
            switch (_bmp085Mode)
            {
                case (byte)bmp085_mode_t.BMP085_MODE_ULTRALOWPOWER:
                    Task.Delay(5).Wait();
                    break;
                case (byte)bmp085_mode_t.BMP085_MODE_STANDARD:
                    Task.Delay(8).Wait();
                    break;
                case (byte)bmp085_mode_t.BMP085_MODE_HIGHRES:
                    Task.Delay(14).Wait();
                    break;
                case (byte)bmp085_mode_t.BMP085_MODE_ULTRAHIGHRES:
                default:
                    Task.Delay(26).Wait();
                    break;
            }

            read16(BMP085_REGISTER_PRESSUREDATA, ref p16);
            p32 = (int)((uint)p16 << 8);
            read8(BMP085_REGISTER_PRESSUREDATA+2, ref p8);
            p32 += p8;
            p32 >>= (8 - _bmp085Mode);
    
            pressure = p32;
        }

        /**************************************************************************/
        /*!
            @brief  Compute B5 coefficient used in temperature & pressure calcs.
        */
        /**************************************************************************/
        private int computeB5(int ut)
        {
            int X1 = (ut - (int)_bmp085_coeffs.ac6) * ((int)_bmp085_coeffs.ac5) >> 15;
            int X2 = ((int)_bmp085_coeffs.mc << 11) / (X1 + (int)_bmp085_coeffs.md);
            return X1 + X2;
        }


        /***************************************************************************
         CONSTRUCTOR
         ***************************************************************************/

        /**************************************************************************/
        /*!
            @brief  Instantiates a new Adafruit_BMP085_Unified class
        */
        /**************************************************************************/
        public Adafruit_BMP085_Unified(ref I2cDevice I2CDevice, int sensorID) {
        _sensorID = sensorID;
            this.I2CDev = I2CDevice;

}

    /***************************************************************************
     PUBLIC FUNCTIONS
     ***************************************************************************/

    /**************************************************************************/
    /*!
        @brief  Setups the HW
    */
    /**************************************************************************/
    public bool begin(bmp085_mode_t mode=bmp085_mode_t.BMP085_MODE_ULTRAHIGHRES)
    {
        /* Mode boundary check */
        if ((mode > bmp085_mode_t.BMP085_MODE_ULTRAHIGHRES) || (mode < 0))
        {
            mode = bmp085_mode_t.BMP085_MODE_ULTRAHIGHRES;
        }

        /* Make sure we have the right device */
        byte id=0;
        read8(BMP085_REGISTER_CHIPID, ref id);
        if (id != 0x55)
        {
            return false;
        }

        /* Set the mode indicator */
        _bmp085Mode = (byte)mode;

        /* Coefficients need to be read once */
        readCoefficients();

        return true;
    }

    /**************************************************************************/
    /*!
        @brief  Gets the compensated pressure level in kPa
    */
    /**************************************************************************/
   public void getPressure(ref float pressure)
    {
        int ut = 0, up = 0, compp = 0;
        int x1, x2, b5, b6, x3, b3, p;
        uint b4, b7;

        /* Get the raw pressure and temperature values */
        readRawTemperature(ref ut);
        readRawPressure(ref up);

        /* Temperature compensation */
        b5 = computeB5(ut);

        /* Pressure compensation */
        b6 = b5 - 4000;
        x1 = (_bmp085_coeffs.b2 * ((b6 * b6) >> 12)) >> 11;
        x2 = (_bmp085_coeffs.ac2 * b6) >> 11;
        x3 = x1 + x2;
        b3 = (((((int)_bmp085_coeffs.ac1) * 4 + x3) << _bmp085Mode) + 2) >> 2;
        x1 = (_bmp085_coeffs.ac3 * b6) >> 13;
        x2 = (_bmp085_coeffs.b1 * ((b6 * b6) >> 12)) >> 16;
        x3 = ((x1 + x2) + 2) >> 2;
        b4 = (_bmp085_coeffs.ac4 * (uint)(x3 + 32768)) >> 15;
        b7 = ((uint)(up - b3) * ((uint)50000 >> _bmp085Mode));

        if (b7 < 0x80000000)
        {
            p = (int)((b7 << 1) / b4);
        }
        else
        {
            p = (int)((b7 / b4) << 1);
        }

        x1 = (p >> 8) * (p >> 8);
        x1 = (x1 * 3038) >> 16;
        x2 = (-7357 * p) >> 16;
        compp = p + ((x1 + x2 + 3791) >> 4);

        /* Assign compensated pressure value */
        pressure = compp;
    }

    /**************************************************************************/
    /*!
        @brief  Reads the temperatures in degrees Celsius
    */
    /**************************************************************************/
    public void getTemperature(ref float temp)
    {
        int UT=0, B5;     // following ds convention
        float t;

        readRawTemperature(ref UT);

        B5 = computeB5(UT);
        t = (B5 + 8) >> 4;
        t /= 10;

        temp = t;
    }

    /**************************************************************************/
    /*!
        Calculates the altitude (in meters) from the specified atmospheric
        pressure (in hPa), and sea-level pressure (in hPa).

        @param  seaLevel      Sea-level pressure in hPa
        @param  atmospheric   Atmospheric pressure in hPa
    */
    /**************************************************************************/
    public float pressureToAltitude(float seaLevel, float atmospheric)
    {
        // Equation taken from BMP180 datasheet (page 16):
        //  http://www.adafruit.com/datasheets/BST-BMP180-DS000-09.pdf

        // Note that using the equation from wikipedia can give bad results
        // at high altitude.  See this thread for more information:
        //  http://forums.adafruit.com/viewtopic.php?f=22&t=58064

        return (float)(44330.0 * (1.0 - Math.Pow((atmospheric / seaLevel), 0.1903)));
    }

    /**************************************************************************/
    /*!
        Calculates the altitude (in meters) from the specified atmospheric
        pressure (in hPa), and sea-level pressure (in hPa).  Note that this
        function just calls the overload of pressureToAltitude which takes
        seaLevel and atmospheric pressure--temperature is ignored.  The original
        implementation of this function was based on calculations from Wikipedia
        which are not accurate at higher altitudes.  To keep compatibility with
        old code this function remains with the same interface, but it calls the
        more accurate calculation.

        @param  seaLevel      Sea-level pressure in hPa
        @param  atmospheric   Atmospheric pressure in hPa
        @param  temp          Temperature in degrees Celsius
    */
    /**************************************************************************/
    public float pressureToAltitude(float seaLevel, float atmospheric, float temp)
    {
        return pressureToAltitude(seaLevel, atmospheric);
    }

    /**************************************************************************/
    /*!
        Calculates the pressure at sea level (in hPa) from the specified altitude 
        (in meters), and atmospheric pressure (in hPa).  

        @param  altitude      Altitude in meters
        @param  atmospheric   Atmospheric pressure in hPa
    */
    /**************************************************************************/
    public float seaLevelForAltitude(float altitude, float atmospheric)
    {
        // Equation taken from BMP180 datasheet (page 17):
        //  http://www.adafruit.com/datasheets/BST-BMP180-DS000-09.pdf

        // Note that using the equation from wikipedia can give bad results
        // at high altitude.  See this thread for more information:
        //  http://forums.adafruit.com/viewtopic.php?f=22&t=58064

        return (float)(atmospheric / Math.Pow(1.0 - (altitude / 44330.0), 5.255));
    }

    /**************************************************************************/
    /*!
        Calculates the pressure at sea level (in hPa) from the specified altitude 
        (in meters), and atmospheric pressure (in hPa).  Note that this
        function just calls the overload of seaLevelForAltitude which takes
        altitude and atmospheric pressure--temperature is ignored.  The original
        implementation of this function was based on calculations from Wikipedia
        which are not accurate at higher altitudes.  To keep compatibility with
        old code this function remains with the same interface, but it calls the
        more accurate calculation.

        @param  altitude      Altitude in meters
        @param  atmospheric   Atmospheric pressure in hPa
        @param  temp          Temperature in degrees Celsius
    */
    /**************************************************************************/
    public float seaLevelForAltitude(float altitude, float atmospheric, float temp)
    {
        return seaLevelForAltitude(altitude, atmospheric);
    }

    public sensor_t getSensorObj()
    {
        return Sensor;
    }
    public sensors_event_t getSensorEventObj()
    {
        return SensEvent;
    }

    /**************************************************************************/
    /*!
        @brief  Provides the sensor_t data for this sensor
    */
    /**************************************************************************/
    public void getSensor(ref sensor_t sensor)
    {
       
            /* Insert the sensor name in the fixed length char array */
        sensor.name="BMP085";
        sensor.version = 1;
        sensor.sensor_id = _sensorID;
            sensor.type = 6;    // SENSOR_TYPE_PRESSURE;
        sensor.min_delay = 0;
        sensor.max_value = 300.0F;               // 300..1100 hPa
        sensor.min_value = 1100.0F;
        sensor.resolution = 0.01F;                // Datasheet states 0.01 hPa resolution
    }

    /**************************************************************************/
    /*!
        @brief  Reads the sensor and returns the data as a sensors_event_t
    */
    /**************************************************************************/

    public void getEvent(ref sensors_event_t Event)
    {
          float pressure_kPa = 0;
          Event.type = 6; //SENSOR_TYPE_PRESSURE;
          Event.timestamp = 0;
          Event.sensor_id = _sensorID;
          getPressure(ref pressure_kPa);
          Event.pressure = pressure_kPa / 100.0F;
    }


}
}