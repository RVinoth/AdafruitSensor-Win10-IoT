using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Windows.Devices.I2c;
namespace AdafruitSensor
{
    class Adafruit_L3GD20_Unified
    {
        // I2C Device
        private I2cDevice I2CDev;

        /*=========================================================================
    I2C ADDRESS/BITS AND SETTINGS
    -----------------------------------------------------------------------*/
        public const int L3GD20_ADDRESS =        (0x6B);        // 1101011
        private const byte L3GD20_POLL_TIMEOUT = (100);         // Maximum number of read attempts
        private const byte L3GD20_ID = (0xD4);         //(0b11010100)
        private const float GYRO_SENSITIVITY_250DPS = (0.00875F);    // Roughly 22/256 for fixed point match
        private const float GYRO_SENSITIVITY_500DPS = (0.0175F);     // Roughly 45/256
        private const float GYRO_SENSITIVITY_2000DPS = (0.070F);      // Roughly 18/256
        private const float SENSORS_DPS_TO_RADS = (0.017453293F);          /*< Degrees/s to rad/s multiplier */
        /*=========================================================================*/

        /*=========================================================================
            REGISTERS
            -----------------------------------------------------------------------*/
        private enum gyroRegisters_t
        {                                             // DEFAULT    TYPE
            GYRO_REGISTER_WHO_AM_I = 0x0F,   // 11010100   r
            GYRO_REGISTER_CTRL_REG1 = 0x20,   // 00000111   rw
            GYRO_REGISTER_CTRL_REG2 = 0x21,   // 00000000   rw
            GYRO_REGISTER_CTRL_REG3 = 0x22,   // 00000000   rw
            GYRO_REGISTER_CTRL_REG4 = 0x23,   // 00000000   rw
            GYRO_REGISTER_CTRL_REG5 = 0x24,   // 00000000   rw
            GYRO_REGISTER_REFERENCE = 0x25,   // 00000000   rw
            GYRO_REGISTER_OUT_TEMP = 0x26,   //            r
            GYRO_REGISTER_STATUS_REG = 0x27,   //            r
            GYRO_REGISTER_OUT_X_L = 0x28,   //            r
            GYRO_REGISTER_OUT_X_H = 0x29,   //            r
            GYRO_REGISTER_OUT_Y_L = 0x2A,   //            r
            GYRO_REGISTER_OUT_Y_H = 0x2B,   //            r
            GYRO_REGISTER_OUT_Z_L = 0x2C,   //            r
            GYRO_REGISTER_OUT_Z_H = 0x2D,   //            r
            GYRO_REGISTER_FIFO_CTRL_REG = 0x2E,   // 00000000   rw
            GYRO_REGISTER_FIFO_SRC_REG = 0x2F,   //            r
            GYRO_REGISTER_INT1_CFG = 0x30,   // 00000000   rw
            GYRO_REGISTER_INT1_SRC = 0x31,   //            r
            GYRO_REGISTER_TSH_XH = 0x32,   // 00000000   rw
            GYRO_REGISTER_TSH_XL = 0x33,   // 00000000   rw
            GYRO_REGISTER_TSH_YH = 0x34,   // 00000000   rw
            GYRO_REGISTER_TSH_YL = 0x35,   // 00000000   rw
            GYRO_REGISTER_TSH_ZH = 0x36,   // 00000000   rw
            GYRO_REGISTER_TSH_ZL = 0x37,   // 00000000   rw
            GYRO_REGISTER_INT1_DURATION = 0x38    // 00000000   rw
        };    
/*=========================================================================*/

/*=========================================================================
    OPTIONAL SPEED SETTINGS
    -----------------------------------------------------------------------*/
    public enum gyroRange_t
        {
            GYRO_RANGE_250DPS = 250,
            GYRO_RANGE_500DPS = 500,
            GYRO_RANGE_2000DPS = 2000
     };
        /*=========================================================================*/

       private gyroRange_t _range;
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
            public sensors_vec_t gyro;                 /**< gyroscope values are in rad/s */
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

        /***************************************************************************
         CONSTRUCTOR
         ***************************************************************************/

        /**************************************************************************/
        /*!
            @brief  Instantiates a new Adafruit_L3GD20_Unified class
        */
        /**************************************************************************/
        public Adafruit_L3GD20_Unified(ref I2cDevice I2CDevice, int sensorID) {
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
    public bool begin(gyroRange_t rng= gyroRange_t.GYRO_RANGE_250DPS)
    {
        
        /* Set the range the an appropriate value */
        _range = rng;

        /* Make sure we have the correct chip ID since this checks
           for correct address and that the IC is properly connected */
        byte whoami = read8((byte)gyroRegisters_t.GYRO_REGISTER_WHO_AM_I);
        if (whoami != L3GD20_ID)
        {
            return false;
        }

        /* Set CTRL_REG1 (0x20)
         ====================================================================
         BIT  Symbol    Description                                   Default
         ---  ------    --------------------------------------------- -------
         7-6  DR1/0     Output data rate                                   00
         5-4  BW1/0     Bandwidth selection                                00
           3  PD        0 = Power-down mode, 1 = normal/sleep mode          0
           2  ZEN       Z-axis enable (0 = disabled, 1 = enabled)           1
           1  YEN       Y-axis enable (0 = disabled, 1 = enabled)           1
           0  XEN       X-axis enable (0 = disabled, 1 = enabled)           1 */

        /* Reset then switch to normal mode and enable all three channels */
        write8((byte)gyroRegisters_t.GYRO_REGISTER_CTRL_REG1, 0x00);
        write8((byte)gyroRegisters_t.GYRO_REGISTER_CTRL_REG1, 0x0F);
        /* ------------------------------------------------------------------ */

        /* Set CTRL_REG2 (0x21)
         ====================================================================
         BIT  Symbol    Description                                   Default
         ---  ------    --------------------------------------------- -------
         5-4  HPM1/0    High-pass filter mode selection                    00
         3-0  HPCF3..0  High-pass filter cutoff frequency selection      0000 */

        /* Nothing to do ... keep default values */
        /* ------------------------------------------------------------------ */

        /* Set CTRL_REG3 (0x22)
         ====================================================================
         BIT  Symbol    Description                                   Default
         ---  ------    --------------------------------------------- -------
           7  I1_Int1   Interrupt enable on INT1 (0=disable,1=enable)       0
           6  I1_Boot   Boot status on INT1 (0=disable,1=enable)            0
           5  H-Lactive Interrupt active config on INT1 (0=high,1=low)      0
           4  PP_OD     Push-Pull/Open-Drain (0=PP, 1=OD)                   0
           3  I2_DRDY   Data ready on DRDY/INT2 (0=disable,1=enable)        0
           2  I2_WTM    FIFO wtrmrk int on DRDY/INT2 (0=dsbl,1=enbl)        0
           1  I2_ORun   FIFO overrun int on DRDY/INT2 (0=dsbl,1=enbl)       0
           0  I2_Empty  FIFI empty int on DRDY/INT2 (0=dsbl,1=enbl)         0 */

        /* Nothing to do ... keep default values */
        /* ------------------------------------------------------------------ */

        /* Set CTRL_REG4 (0x23)
         ====================================================================
         BIT  Symbol    Description                                   Default
         ---  ------    --------------------------------------------- -------
           7  BDU       Block Data Update (0=continuous, 1=LSB/MSB)         0
           6  BLE       Big/Little-Endian (0=Data LSB, 1=Data MSB)          0
         5-4  FS1/0     Full scale selection                               00
                                        00 = 250 dps
                                        01 = 500 dps
                                        10 = 2000 dps
                                        11 = 2000 dps
           0  SIM       SPI Mode (0=4-wire, 1=3-wire)                       0 */

        /* Adjust resolution if requested */
        switch (_range)
        {
            case gyroRange_t.GYRO_RANGE_250DPS:
                write8((byte)gyroRegisters_t.GYRO_REGISTER_CTRL_REG4, 0x00);
                break;
            case gyroRange_t.GYRO_RANGE_500DPS:
                write8((byte)gyroRegisters_t.GYRO_REGISTER_CTRL_REG4, 0x10);
                break;
            case gyroRange_t.GYRO_RANGE_2000DPS:
                write8((byte)gyroRegisters_t.GYRO_REGISTER_CTRL_REG4, 0x20);
                break;
        }
        /* ------------------------------------------------------------------ */

        /* Set CTRL_REG5 (0x24)
         ====================================================================
         BIT  Symbol    Description                                   Default
         ---  ------    --------------------------------------------- -------
           7  BOOT      Reboot memory content (0=normal, 1=reboot)          0
           6  FIFO_EN   FIFO enable (0=FIFO disable, 1=enable)              0
           4  HPen      High-pass filter enable (0=disable,1=enable)        0
         3-2  INT1_SEL  INT1 Selection config                              00
         1-0  OUT_SEL   Out selection config                               00 */

        /* Nothing to do ... keep default values */
        /* ------------------------------------------------------------------ */

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
        @brief  Gets the most recent sensor event
    */
    /**************************************************************************/
    public void getEvent(ref sensors_event_t Event)
    {
        bool readingValid = false;
        
        Event.sensor_id = _sensorID;
        Event.type = 4; // SENSOR_TYPE_GYROSCOPE;

        while (!readingValid)
        {
            Event.timestamp = 0;
            byte[] addr = new byte[] { (byte)((byte)gyroRegisters_t.GYRO_REGISTER_OUT_X_L | 0x80) };
            byte[] data = new byte[6];
            I2CDev.WriteRead(addr, data);
            
            byte xlo = data[0];
            byte xhi = data[1];
            byte ylo = data[2];
            byte yhi = data[3];
            byte zlo = data[4];
            byte zhi = data[5];
        
            /* Shift values to create properly formed integer (low byte first) */
            Event.gyro.x = (short)(xlo | (xhi << 8));
            Event.gyro.y = (short)(ylo | (yhi << 8));
            Event.gyro.z = (short)(zlo | (zhi << 8));

            /* Make sure the sensor isn't saturating if auto-ranging is enabled */
            if (!_autoRangeEnabled)
            {
                readingValid = true;
            }
            else
            {
                /* Check if the sensor is saturating or not */
                if ((Event.gyro.x >= 32760) | (Event.gyro.x <= -32760) |
                    (Event.gyro.y >= 32760) | (Event.gyro.y <= -32760) |
                    (Event.gyro.z >= 32760) | (Event.gyro.z <= -32760) )
                {
                    /* Saturating .... increase the range if we can */
                    switch (_range)
                    {
                        case gyroRange_t.GYRO_RANGE_500DPS:
                            /* Push the range up to 2000dps */
                            _range = gyroRange_t.GYRO_RANGE_2000DPS;
                            write8((byte)gyroRegisters_t.GYRO_REGISTER_CTRL_REG1, 0x00);
                            write8((byte)gyroRegisters_t.GYRO_REGISTER_CTRL_REG1, 0x0F);
                            write8((byte)gyroRegisters_t.GYRO_REGISTER_CTRL_REG4, 0x20);
                            write8((byte)gyroRegisters_t.GYRO_REGISTER_CTRL_REG5, 0x80);
                            readingValid = false;
                            // Serial.println("Changing range to 2000DPS");
                            break;
                        case gyroRange_t.GYRO_RANGE_250DPS:
                            /* Push the range up to 500dps */
                            _range = gyroRange_t.GYRO_RANGE_500DPS;
                            write8((byte)gyroRegisters_t.GYRO_REGISTER_CTRL_REG1, 0x00);
                            write8((byte)gyroRegisters_t.GYRO_REGISTER_CTRL_REG1, 0x0F);
                            write8((byte)gyroRegisters_t.GYRO_REGISTER_CTRL_REG4, 0x10);
                            write8((byte)gyroRegisters_t.GYRO_REGISTER_CTRL_REG5, 0x80);
                            readingValid = false;
                            // Serial.println("Changing range to 500DPS");
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

        /* Compensate values depending on the resolution */
        switch (_range)
        {
            case gyroRange_t.GYRO_RANGE_250DPS:
                    Event.gyro.x *= GYRO_SENSITIVITY_250DPS;
                    Event.gyro.y *= GYRO_SENSITIVITY_250DPS;
                    Event.gyro.z *= GYRO_SENSITIVITY_250DPS;
                break;
            case gyroRange_t.GYRO_RANGE_500DPS:
                    Event.gyro.x *= GYRO_SENSITIVITY_500DPS;
                    Event.gyro.y *= GYRO_SENSITIVITY_500DPS;
                    Event.gyro.z *= GYRO_SENSITIVITY_500DPS;
                break;
            case gyroRange_t.GYRO_RANGE_2000DPS:
                    Event.gyro.x *= GYRO_SENSITIVITY_2000DPS;
                    Event.gyro.y *= GYRO_SENSITIVITY_2000DPS;
                    Event.gyro.z *= GYRO_SENSITIVITY_2000DPS;
                break;
        }

            /* Convert values to rad/s */
            Event.gyro.x *= SENSORS_DPS_TO_RADS;
            Event.gyro.y *= SENSORS_DPS_TO_RADS;
            Event.gyro.z *= SENSORS_DPS_TO_RADS;
    }

    /**************************************************************************/
    /*! 
        @brief  Gets the sensor_t data
    */
    /**************************************************************************/
    public void getSensor(ref sensor_t sensor)
    {
       
        sensor.name = "L3GD20";
        sensor.version = 1;
        sensor.sensor_id = _sensorID;
        sensor.type = 4; // SENSOR_TYPE_GYROSCOPE;
        sensor.min_delay = 0;
        sensor.max_value = (float)this._range * SENSORS_DPS_TO_RADS;
        sensor.min_value = ((float)this._range * -1.0f) * SENSORS_DPS_TO_RADS;
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
