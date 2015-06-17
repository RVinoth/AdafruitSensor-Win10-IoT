using System;
using System.Diagnostics;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Windows.Devices.I2c;

namespace AdafruitSensor
{
    /***************************************************************************
    ACCELEROMETER
    ***************************************************************************/
    class Adafruit_LSM303_Accel_Unified
    {
        // I2C Device
        private I2cDevice I2CDev;
        /*=========================================================================
        I2C ADDRESS/BITS
        -----------------------------------------------------------------------*/
        public const byte LSM303_ADDRESS_ACCEL = (0x32 >> 1);         // 0011001x
        private const float SENSORS_GRAVITY_EARTH = (9.80665F);              /**< Earth's gravity in m/s^2 */
        private const float SENSORS_GRAVITY_MOON = (1.6F);                  /**< The moon's gravity in m/s^2 */
        private const float SENSORS_GRAVITY_SUN = (275.0F);                /**< The sun's gravity in m/s^2 */
        private const float SENSORS_GRAVITY_STANDARD = (SENSORS_GRAVITY_EARTH);

        /*=========================================================================
        REGISTERS
        -----------------------------------------------------------------------*/
        private enum lsm303AccelRegisters_t
        {                                                     // DEFAULT    TYPE
            LSM303_REGISTER_ACCEL_CTRL_REG1_A = 0x20,   // 00000111   rw
            LSM303_REGISTER_ACCEL_CTRL_REG2_A = 0x21,   // 00000000   rw
            LSM303_REGISTER_ACCEL_CTRL_REG3_A = 0x22,   // 00000000   rw
            LSM303_REGISTER_ACCEL_CTRL_REG4_A = 0x23,   // 00000000   rw
            LSM303_REGISTER_ACCEL_CTRL_REG5_A = 0x24,   // 00000000   rw
            LSM303_REGISTER_ACCEL_CTRL_REG6_A = 0x25,   // 00000000   rw
            LSM303_REGISTER_ACCEL_REFERENCE_A = 0x26,   // 00000000   r
            LSM303_REGISTER_ACCEL_STATUS_REG_A = 0x27,   // 00000000   r
            LSM303_REGISTER_ACCEL_OUT_X_L_A = 0x28,
            LSM303_REGISTER_ACCEL_OUT_X_H_A = 0x29,
            LSM303_REGISTER_ACCEL_OUT_Y_L_A = 0x2A,
            LSM303_REGISTER_ACCEL_OUT_Y_H_A = 0x2B,
            LSM303_REGISTER_ACCEL_OUT_Z_L_A = 0x2C,
            LSM303_REGISTER_ACCEL_OUT_Z_H_A = 0x2D,
            LSM303_REGISTER_ACCEL_FIFO_CTRL_REG_A = 0x2E,
            LSM303_REGISTER_ACCEL_FIFO_SRC_REG_A = 0x2F,
            LSM303_REGISTER_ACCEL_INT1_CFG_A = 0x30,
            LSM303_REGISTER_ACCEL_INT1_SOURCE_A = 0x31,
            LSM303_REGISTER_ACCEL_INT1_THS_A = 0x32,
            LSM303_REGISTER_ACCEL_INT1_DURATION_A = 0x33,
            LSM303_REGISTER_ACCEL_INT2_CFG_A = 0x34,
            LSM303_REGISTER_ACCEL_INT2_SOURCE_A = 0x35,
            LSM303_REGISTER_ACCEL_INT2_THS_A = 0x36,
            LSM303_REGISTER_ACCEL_INT2_DURATION_A = 0x37,
            LSM303_REGISTER_ACCEL_CLICK_CFG_A = 0x38,
            LSM303_REGISTER_ACCEL_CLICK_SRC_A = 0x39,
            LSM303_REGISTER_ACCEL_CLICK_THS_A = 0x3A,
            LSM303_REGISTER_ACCEL_TIME_LIMIT_A = 0x3B,
            LSM303_REGISTER_ACCEL_TIME_LATENCY_A = 0x3C,
            LSM303_REGISTER_ACCEL_TIME_WINDOW_A = 0x3D
        };
        /*=========================================================================*/
        /*=========================================================================
           INTERNAL ACCELERATION DATA TYPE
           -----------------------------------------------------------------------*/
    public struct lsm303AccelData
        {
            public float x;
            public float y;
            public float z;
        };
        /*=========================================================================*/

        private lsm303AccelData _accelData;   // Last read accelerometer data will be available here
        private int _sensorID;

        /** struct sensors_vec_s is used to return a vector in a common format. */
        public struct sensors_vec_t
        {

            public float x;
            public float y;
            public float z;
            public byte status;
        };


        /* Sensor event (36 bytes) */
        /** struct sensor_event_s is used to provide a single sensor event in a common format. */
        public struct sensors_event_t
        {
            public int version;                          /**< must be sizeof(struct sensors_event_t) */
            public int sensor_id;                        /**< unique sensor identifier */
            public int type;                             /**< sensor type */
            public int reserved0;                        /**< reserved */
            public int timestamp;                        /**< time is in milliseconds */
            public sensors_vec_t acceleration;                 /**< gyroscope values are in rad/s */
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

        private const float _lsm303Accel_MG_LSB = 0.001F;   // 1, 2, 4 or 12 mg per lsb

        /***************************************************************************
        PRIVATE FUNCTIONS
        ***************************************************************************/

        /**************************************************************************/
        /*!
            @brief  Abstract away platform differences in Arduino wire library
        */
        /**************************************************************************/
        private void write8(byte reg, byte value)
        {
            I2CDev.Write(new byte[] { reg, value });
        }

        /**************************************************************************/
        /*!
            @brief  Abstract away platform differences in Arduino wire library
        */
        /**************************************************************************/
        private byte read8(byte reg)
        {
            byte value;
            byte[] addr = new byte[] { reg };
            byte[] data = new byte[1];
            I2CDev.WriteRead(addr, data);
            value = data[0];
            return value;
        }

        /**************************************************************************/
        /*!
            @brief  Reads the raw data from the sensor
        */
        /**************************************************************************/
        private void read()
        {
            byte[] addr = new byte[] { (byte)((byte)lsm303AccelRegisters_t.LSM303_REGISTER_ACCEL_OUT_X_L_A | 0x80) };
            byte[] data = new byte[6];
            I2CDev.WriteRead(addr, data);

            byte xlo = data[0];
            byte xhi = data[1];
            byte ylo = data[2];
            byte yhi = data[3];
            byte zlo = data[4];
            byte zhi = data[5];
           
            // Shift values to create properly formed integer (low byte first)
            _accelData.x = (short)(xlo | (xhi << 8)) >> 4;
            _accelData.y = (short)(ylo | (yhi << 8)) >> 4;
            _accelData.z = (short)(zlo | (zhi << 8)) >> 4;
        }

        /***************************************************************************
         CONSTRUCTOR
         ***************************************************************************/

        /**************************************************************************/
        /*!
            @brief  Instantiates a new Adafruit_LSM303 class
        */
        /**************************************************************************/
        public Adafruit_LSM303_Accel_Unified(ref I2cDevice I2CDevice,int sensorID = -1) {
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
        public bool begin()
        {
     
            // Enable the accelerometer (100Hz)
            write8((byte)lsm303AccelRegisters_t.LSM303_REGISTER_ACCEL_CTRL_REG1_A, 0x57);

            // LSM303DLHC has no WHOAMI register so read CTRL_REG1_A back to check
            // if we are connected or not
            byte reg1_a = read8((byte)lsm303AccelRegisters_t.LSM303_REGISTER_ACCEL_CTRL_REG1_A);
            if (reg1_a != 0x57)
            {
                return false;
            }

            return true;
        }

        /**************************************************************************/
        /*! 
            @brief  Gets the most recent sensor event
        */
        /**************************************************************************/
        public void getEvent(ref sensors_event_t Event)
        {
        
            /* Read new data */
            read();

            Event.version = 1;
            Event.sensor_id = _sensorID;
            Event.type = 1; //SENSOR_TYPE_ACCELEROMETER;
            Event.timestamp = 0;
            Event.acceleration.x = _accelData.x * _lsm303Accel_MG_LSB * SENSORS_GRAVITY_STANDARD;
            Event.acceleration.y = _accelData.y * _lsm303Accel_MG_LSB * SENSORS_GRAVITY_STANDARD;
            Event.acceleration.z = _accelData.z * _lsm303Accel_MG_LSB * SENSORS_GRAVITY_STANDARD;
        }

        /**************************************************************************/
        /*! 
            @brief  Gets the sensor_t data
        */
        /**************************************************************************/
        public void getSensor(ref sensor_t sensor)
        {
                /* Insert the sensor name in the fixed length char array */
                sensor.name = "LSM303";
                sensor.version = 1;
                sensor.sensor_id = _sensorID;
                sensor.type = 1;    // SENSOR_TYPE_ACCELEROMETER;
                sensor.min_delay = 0;
                sensor.max_value = 0.0F; // TBD
                sensor.min_value = 0.0F; // TBD
                sensor.resolution = 0.0F; // TBD
        }

        public sensor_t getSensorObj()
        {
            return Sensor;
        }
        public sensors_event_t getSensorEventObj()
        {
            return SensEvent;
        }


    }

    class Adafruit_LSM303_Mag_Unified
    {
        // I2C Device
        private I2cDevice I2CDev;
        /*=========================================================================
        I2C ADDRESS/BITS
        -----------------------------------------------------------------------*/
        public const byte LSM303_ADDRESS_MAG = (0x3C >> 1);         // 0011110x
        private const float SENSORS_MAGFIELD_EARTH_MAX = (60.0F);                 /**< Maximum magnetic field on Earth's surface */
        private const float SENSORS_MAGFIELD_EARTH_MIN = (30.0F);                 /**< Minimum magnetic field on Earth's surface */
        private const byte SENSORS_GAUSS_TO_MICROTESLA = (100);                   /**< Gauss to micro-Tesla multiplier */
        /*=========================================================================
        REGISTERS
        -----------------------------------------------------------------------*/
        private enum lsm303MagRegisters_t
        {
            LSM303_REGISTER_MAG_CRA_REG_M = 0x00,
            LSM303_REGISTER_MAG_CRB_REG_M = 0x01,
            LSM303_REGISTER_MAG_MR_REG_M = 0x02,
            LSM303_REGISTER_MAG_OUT_X_H_M = 0x03,
            LSM303_REGISTER_MAG_OUT_X_L_M = 0x04,
            LSM303_REGISTER_MAG_OUT_Z_H_M = 0x05,
            LSM303_REGISTER_MAG_OUT_Z_L_M = 0x06,
            LSM303_REGISTER_MAG_OUT_Y_H_M = 0x07,
            LSM303_REGISTER_MAG_OUT_Y_L_M = 0x08,
            LSM303_REGISTER_MAG_SR_REG_Mg = 0x09,
            LSM303_REGISTER_MAG_IRA_REG_M = 0x0A,
            LSM303_REGISTER_MAG_IRB_REG_M = 0x0B,
            LSM303_REGISTER_MAG_IRC_REG_M = 0x0C,
            LSM303_REGISTER_MAG_TEMP_OUT_H_M = 0x31,
            LSM303_REGISTER_MAG_TEMP_OUT_L_M = 0x32
        };

        /*=========================================================================
    MAGNETOMETER GAIN SETTINGS
    -----------------------------------------------------------------------*/
        public enum lsm303MagGain
        {
            LSM303_MAGGAIN_1_3 = 0x20,  // +/- 1.3
            LSM303_MAGGAIN_1_9 = 0x40,  // +/- 1.9
            LSM303_MAGGAIN_2_5 = 0x60,  // +/- 2.5
            LSM303_MAGGAIN_4_0 = 0x80,  // +/- 4.0
            LSM303_MAGGAIN_4_7 = 0xA0,  // +/- 4.7
            LSM303_MAGGAIN_5_6 = 0xC0,  // +/- 5.6
            LSM303_MAGGAIN_8_1 = 0xE0   // +/- 8.1
        };	
/*=========================================================================*/

/*=========================================================================
    INTERNAL MAGNETOMETER DATA TYPE
    -----------------------------------------------------------------------*/
    public struct lsm303MagData
        {
            public float x;
            public float y;
            public float z;
            public float orientation;
        };
        /*=========================================================================*/

        private lsm303MagGain _magGain;
        private lsm303MagData _magData;     // Last read magnetometer data will be available here
        private int _sensorID;
        private bool _autoRangeEnabled;

        /** struct sensors_vec_s is used to return a vector in a common format. */
        public struct sensors_vec_t
        {

            public float x;
            public float y;
            public float z;
            public byte status;
        };


        /* Sensor event (36 bytes) */
        /** struct sensor_event_s is used to provide a single sensor event in a common format. */
        public struct sensors_event_t
        {
            public int version;                          /**< must be sizeof(struct sensors_event_t) */
            public int sensor_id;                        /**< unique sensor identifier */
            public int type;                             /**< sensor type */
            public int reserved0;                        /**< reserved */
            public int timestamp;                        /**< time is in milliseconds */
            public sensors_vec_t magnetic;                 /**< gyroscope values are in rad/s */
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

        private  float _lsm303Mag_Gauss_LSB_XY = 1100.0F;  // Varies with gain
        private  float _lsm303Mag_Gauss_LSB_Z = 980.0F;   // Varies with gain


        /***************************************************************************
         MAGNETOMETER
         ***************************************************************************/
        /***************************************************************************
         PRIVATE FUNCTIONS
         ***************************************************************************/

        /**************************************************************************/
        /*!
            @brief  Abstract away platform differences in Arduino wire library
        */
        private void write8(byte reg, byte value)
        {
            I2CDev.Write(new byte[] { reg, value });
        }

        /**************************************************************************/
        /*!
            @brief  Abstract away platform differences in Arduino wire library
        */
        /**************************************************************************/
        private byte read8(byte reg)
        {
            byte value;
            byte[] addr = new byte[] { reg };
            byte[] data = new byte[1];
            I2CDev.WriteRead(addr, data);
            value = data[0];
            return value;
        }

        /**************************************************************************/
        /*!
            @brief  Reads the raw data from the sensor
        */
        /**************************************************************************/
        private void read()
        {
            byte[] addr = new byte[] { (byte)((byte)lsm303MagRegisters_t.LSM303_REGISTER_MAG_OUT_X_H_M) };
            byte[] data = new byte[6];
            I2CDev.WriteRead(addr, data);

            byte xlo = data[0];
            byte xhi = data[1];
            byte ylo = data[2];
            byte yhi = data[3];
            byte zlo = data[4];
            byte zhi = data[5];

            // Shift values to create properly formed integer (low byte first)
            _magData.x = (short)(xlo | ((short)xhi << 8));
            _magData.y = (short)(ylo | ((short)yhi << 8));
            _magData.z = (short)(zlo | ((short)zhi << 8));

            // ToDo: Calculate orientation
            _magData.orientation = 0.0f;
           
        }
     
        /***************************************************************************
         CONSTRUCTOR
         ***************************************************************************/

        /**************************************************************************/
        /*!
            @brief  Instantiates a new Adafruit_LSM303 class
        */
        /**************************************************************************/
        public Adafruit_LSM303_Mag_Unified(ref I2cDevice I2CDevice,int sensorID = -1) {
            _sensorID = sensorID;
            _autoRangeEnabled = false;
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
    public bool begin()
    {
        // Enable the magnetometer
        write8((byte)lsm303MagRegisters_t.LSM303_REGISTER_MAG_MR_REG_M, 0x00);

        // LSM303DLHC has no WHOAMI register so read CRA_REG_M to check
        // the default value (0b00010000/0x10)
        byte  reg1_a = read8((byte)lsm303MagRegisters_t.LSM303_REGISTER_MAG_CRA_REG_M);
        if (reg1_a != 0x10)
        {
            return false;
        }

        // Set the gain to a known level
        setMagGain(lsm303MagGain.LSM303_MAGGAIN_1_3);

        return true;
    }

    /**************************************************************************/
    /*! 
        @brief  Enables or disables auto-ranging
    */
    /**************************************************************************/
    public void enableAutoRange(bool enabled)
    {
        _autoRangeEnabled = enabled;
    }

    /**************************************************************************/
    /*!
        @brief  Sets the magnetometer's gain
    */
    /**************************************************************************/
    private void setMagGain(lsm303MagGain gain)
    {
        write8((byte)lsm303MagRegisters_t.LSM303_REGISTER_MAG_CRB_REG_M, (byte)gain);

        _magGain = gain;

        switch (gain)
        {
            case lsm303MagGain.LSM303_MAGGAIN_1_3:
                _lsm303Mag_Gauss_LSB_XY = 1100;
                _lsm303Mag_Gauss_LSB_Z = 980;
                break;
            case lsm303MagGain.LSM303_MAGGAIN_1_9:
                _lsm303Mag_Gauss_LSB_XY = 855;
                _lsm303Mag_Gauss_LSB_Z = 760;
                break;
            case lsm303MagGain.LSM303_MAGGAIN_2_5:
                _lsm303Mag_Gauss_LSB_XY = 670;
                _lsm303Mag_Gauss_LSB_Z = 600;
                break;
            case lsm303MagGain.LSM303_MAGGAIN_4_0:
                _lsm303Mag_Gauss_LSB_XY = 450;
                _lsm303Mag_Gauss_LSB_Z = 400;
                break;
            case lsm303MagGain.LSM303_MAGGAIN_4_7:
                _lsm303Mag_Gauss_LSB_XY = 400;
                _lsm303Mag_Gauss_LSB_Z = 255;
                break;
            case lsm303MagGain.LSM303_MAGGAIN_5_6:
                _lsm303Mag_Gauss_LSB_XY = 330;
                _lsm303Mag_Gauss_LSB_Z = 295;
                break;
            case lsm303MagGain.LSM303_MAGGAIN_8_1:
                _lsm303Mag_Gauss_LSB_XY = 230;
                _lsm303Mag_Gauss_LSB_Z = 205;
                break;
        }
    }

    /**************************************************************************/
    /*! 
        @brief  Gets the most recent sensor event
    */
    /**************************************************************************/
    public void getEvent(ref sensors_event_t Event)
    {
        bool readingValid = false;

        while (!readingValid)
        {
            /* Read new data */
            read();

            /* Make sure the sensor isn't saturating if auto-ranging is enabled */
            if (!_autoRangeEnabled)
            {
                readingValid = true;
            }
            else
            {
                    Debug.WriteLine(_magData.x + " " + _magData.y + " " + _magData.z + " ");

                    /* Check if the sensor is saturating or not */
                    if ((_magData.x >= 4090) | (_magData.x <= -4090) |
                     (_magData.y >= 4090) | (_magData.y <= -4090) |
                     (_magData.z >= 4090) | (_magData.z <= -4090))
                {
                    /* Saturating .... increase the range if we can */
                    switch (_magGain)
                    {
                        case lsm303MagGain.LSM303_MAGGAIN_5_6:
                            setMagGain(lsm303MagGain.LSM303_MAGGAIN_8_1);
                            readingValid = false;
                            Debug.WriteLine("Changing range to +/- 8.1");
                            break;
                        case lsm303MagGain.LSM303_MAGGAIN_4_7:
                            setMagGain(lsm303MagGain.LSM303_MAGGAIN_5_6);
                            readingValid = false;
                            Debug.WriteLine("Changing range to +/- 5.6");
                            break;
                        case lsm303MagGain.LSM303_MAGGAIN_4_0:
                            setMagGain(lsm303MagGain.LSM303_MAGGAIN_4_7);
                            readingValid = false;
                            Debug.WriteLine("Changing range to +/- 4.7");
                            break;
                        case lsm303MagGain.LSM303_MAGGAIN_2_5:
                            setMagGain(lsm303MagGain.LSM303_MAGGAIN_4_0);
                            readingValid = false;
                            Debug.WriteLine("Changing range to +/- 4.0");
                            break;
                        case lsm303MagGain.LSM303_MAGGAIN_1_9:
                            setMagGain(lsm303MagGain.LSM303_MAGGAIN_2_5);
                            readingValid = false;
                            Debug.WriteLine("Changing range to +/- 2.5");
                            break;
                        case lsm303MagGain.LSM303_MAGGAIN_1_3:
                            setMagGain(lsm303MagGain.LSM303_MAGGAIN_1_9);
                            readingValid = false;
                            Debug.WriteLine("Changing range to +/- 1.9");
                            break;
                        default:
                            readingValid = true;
                            break;
                    }
                }
                else
                {
                    /* All values are withing range */
                    readingValid = true;
                }
            }
        }
  
      Event.version = 1;
      Event.sensor_id = _sensorID;
      Event.type = 2; // SENSOR_TYPE_MAGNETIC_FIELD;
      Event.timestamp = 0;
      Event.magnetic.x = _magData.x / _lsm303Mag_Gauss_LSB_XY * SENSORS_GAUSS_TO_MICROTESLA;
      Event.magnetic.y = _magData.y / _lsm303Mag_Gauss_LSB_XY * SENSORS_GAUSS_TO_MICROTESLA;
      Event.magnetic.z = _magData.z / _lsm303Mag_Gauss_LSB_Z * SENSORS_GAUSS_TO_MICROTESLA;
    }

    /**************************************************************************/
    /*! 
        @brief  Gets the sensor_t data
    */
    /**************************************************************************/
    public void getSensor(ref sensor_t sensor)
    {
        /* Insert the sensor name in the fixed length char array */
        sensor.name = "LSM303";
        sensor.version = 1;
        sensor.sensor_id = _sensorID;
        sensor.type = 2; //SENSOR_TYPE_MAGNETIC_FIELD;
        sensor.min_delay = 0;
        sensor.max_value = 0.0F; // TBD
        sensor.min_value = 0.0F; // TBD
        sensor.resolution = 0.0F; // TBD
    }


    public sensor_t getSensorObj()
    {
        return Sensor;
    }
    public sensors_event_t getSensorEventObj()
    {
        return SensEvent;
    }

    }
}
