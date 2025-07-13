#ifndef ICM42688_H
#define ICM42688_H

#include <driver/i2c_master.h>
#include "esp_log.h"
#include <string.h>
#include <math.h>
#include <time.h>
#include <sys/time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "cordic.hpp"
#include "kalman.h"
#include "HAL_Def.h"

class ICM42688 {
 public:
	enum GyroFS : uint8_t {
		dps2000   = 0x00,
		dps1000   = 0x01,
		dps500    = 0x02,
		dps250    = 0x03,
		dps125    = 0x04,
		dps62_5   = 0x05,
		dps31_25  = 0x06,
		dps15_625 = 0x07
	};

	enum AccelFS : uint8_t {
		gpm16 = 0x00,
		gpm8  = 0x01,
		gpm4  = 0x02,
		gpm2  = 0x03
	};

	enum ODR : uint8_t {
		odr32k    = 0x01,  // LN mode only
		odr16k    = 0x02,  // LN mode only
		odr8k     = 0x03,  // LN mode only
		odr4k     = 0x04,  // LN mode only
		odr2k     = 0x05,  // LN mode only
		odr1k     = 0x06,  // LN mode only
		odr200    = 0x07,
		odr100    = 0x08,
		odr50     = 0x09,
		odr25     = 0x0A,
		odr12_5   = 0x0B,
		odr6a25   = 0x0C,  // LP mode only (accel only)
		odr3a125  = 0x0D,  // LP mode only (accel only)
		odr1a5625 = 0x0E,  // LP mode only (accel only)
		odr500    = 0x0F,
	};

	enum GyroNFBWsel : uint8_t {
		nfBW1449Hz = 0x00,
		nfBW680z   = 0x01,
		nfBW329Hz  = 0x02,
		nfBW162Hz  = 0x03,
		nfBW80Hz   = 0x04,
		nfBW40Hz   = 0x05,
		nfBW20Hz   = 0x06,
		nfBW10Hz   = 0x07,
	};

	enum UIFiltOrd : uint8_t {
		first_order  = 0x00,
		second_order = 0x01,
		third_order  = 0x02,
	};

	struct filter_t {
		float roll;
		float pitch;
	};

	/**
     * @brief      Constructor for I2C communication
     *
     * @param      bus      I2C bus
     * @param[in]  address  Address of ICM 42688-p device
     */
	ICM42688(i2c_master_bus_handle_t i2c_bus, uint8_t address);

	~ICM42688();

	/**
     * @brief      Initialize the device.
     *
     * @return     ret < 0 if error
     */
	int begin();

	/**
     * @brief      Sets the full scale range for the accelerometer
     *
     * @param[in]  fssel  Full scale selection
     *
     * @return     ret < 0 if error
     */
	int setAccelFS(AccelFS fssel);
	int getAccelFS();
	/**
     * @brief      Sets the full scale range for the gyro
     *
     * @param[in]  fssel  Full scale selection
     *
     * @return     ret < 0 if error
     */
	int setGyroFS(GyroFS fssel);
	int getGyroFS();
	/**
     * @brief      Set the ODR for accelerometer
     *
     * @param[in]  odr   Output data rate
     *
     * @return     ret < 0 if error
     */
	int setAccelODR(ODR odr);
	int getAccelODR();

	/**
     * @brief      Set the ODR for gyro
     *
     * @param[in]  odr   Output data rate
     *
     * @return     ret < 0 if error
     */
	int setGyroODR(ODR odr);
	int getGyroODR();

	int setFilters(bool gyroFilters, bool accFilters);

	/**
     * @brief      Enables the data ready interrupt.
     *
     *             - routes UI data ready interrupt to INT1
     *             - push-pull, pulsed, active HIGH interrupts
     *
     * @return     ret < 0 if error
     */
	int enableDataReadyInterrupt();

	/**
     * @brief      Masks the data ready interrupt
     *
     * @return     ret < 0 if error
     */
	int disableDataReadyInterrupt();

	/**
     * @brief      Transfers data from ICM 42688-p to mcu.
     *             Must be called to access new measurements.
     *
     * @return     ret < 0 if error
     */
	int getAGT();
	int getRawAGT();

	/**
     * @brief      Get accelerometer data, per axis
     *
     * @return     Acceleration in g's
     */
	float accX() const { return _acc[0]; }

	float accY() const { return _acc[1]; }

	float accZ() const { return _acc[2]; }

	/**
     * @brief      Get gyro data, per axis
     *
     * @return     Angular velocity in dps
     */
	float gyrX() const { return _gyr[0]; }

	float gyrY() const { return _gyr[1]; }

	float gyrZ() const { return _gyr[2]; }

	/**
     * @brief      Get temperature of gyro die
     *
     * @return     Temperature in Celsius
     */
	float temp() const { return _t; }

	/**
     * @brief      Get accelerometer Raw data, per axis
     *
     * @return     Acceleration in bytes
     */
	int16_t rawAccX() const { return _rawAcc[0]; }

	int16_t rawAccY() const { return _rawAcc[1]; }

	int16_t rawAccZ() const { return _rawAcc[2]; }

	/**
     * @brief      Get gyro raw data, per axis
     *
     * @return     Angular velocity in bytes
     */
	int16_t rawGyrX() const { return _rawGyr[0]; }

	int16_t rawGyrY() const { return _rawGyr[1]; }

	int16_t rawGyrZ() const { return _rawGyr[2]; }

	/**
     * @brief      Get raw temperature of gyro die
     *
     * @return     Temperature in bytes
     */
	int16_t rawTemp() const { return _rawT; }

	/**
     * @brief      Get raw temperature of gyro die
     *
     * @return     Temperature in bytes
     */
	int32_t rawBiasAccX() const { return _rawAccBias[0]; }

	int32_t rawBiasAccY() const { return _rawAccBias[1]; }

	int32_t rawBiasAccZ() const { return _rawAccBias[2]; }

	int32_t rawBiasGyrX() const { return _rawGyrBias[0]; }

	int32_t rawBiasGyrY() const { return _rawGyrBias[1]; }

	int32_t rawBiasGyrZ() const { return _rawGyrBias[2]; }

	int   computeOffsets();
	int   setAllOffsets();                    //Set all Offsets computed
	int   setGyrXOffset(int16_t gyrXoffset);  //#TODO add the getOffset function
	int   setGyrYOffset(int16_t gyrYoffset);
	int   setGyrZOffset(int16_t gyrZoffset);
	int   setAccXOffset(int16_t accXoffset);
	int   setAccYOffset(int16_t accYoffset);
	int   setAccZOffset(int16_t accZoffset);
	float getAccelRes();
	float getGyroRes();
	int   setUIFilterBlock(UIFiltOrd gyroUIFiltOrder, UIFiltOrd accelUIFiltOrder);
	int   setGyroNotchFilter(float gyroNFfreq_x, float gyroNFfreq_y, float gyroNFfreq_z, GyroNFBWsel gyro_nf_bw);  //
	int   selfTest();
	int   testingFunction();

	int   calibrateGyro();
	float getGyroBiasX();
	float getGyroBiasY();
	float getGyroBiasZ();
	void  setGyroBiasX(float bias);
	void  setGyroBiasY(float bias);
	void  setGyroBiasZ(float bias);
	int   calibrateAccel();
	float getAccelBiasX_mss();
	float getAccelScaleFactorX();
	float getAccelBiasY_mss();
	float getAccelScaleFactorY();
	float getAccelBiasZ_mss();
	float getAccelScaleFactorZ();
	void  setAccelCalX(float bias, float scaleFactor);
	void  setAccelCalY(float bias, float scaleFactor);
	void  setAccelCalZ(float bias, float scaleFactor);
	void  setFilter(HAL::FilterType filter_type) { _filter_type = filter_type; };

	esp_err_t complementory_filter();
	esp_err_t filter();

	float getPitch() { return _angle.pitch; };
	float getRoll() { return _angle.roll; };

	float lowPassGyroX(float alpha=0.2)
	{
		gyroXFV = alpha * _gyr[0] + (1 - alpha) * gyroXFV;
		return gyroXFV;
	}

 protected:
	///\brief I2C Communication
	i2c_master_dev_handle_t   _dev;
	size_t                    _numBytes = 0;        // number of bytes received from I2C

	// buffer for reading from sensor
	uint8_t _buffer[15] = {};

	// data buffer
	float _t      = 0.0f;
	float _acc[3] = {};
	float _gyr[3] = {};

	int16_t _rawT      = 0;
	int16_t _rawAcc[3] = {};
	int16_t _rawGyr[3] = {};

	///\brief Raw Gyro and Accelerometer Bias
	int32_t _rawAccBias[3] = {0, 0, 0};
	int32_t _rawGyrBias[3] = {0, 0, 0};

	///\brief Raw Gyro and Accelerometer Offsets
	int16_t _AccOffset[3] = {0, 0, 0};
	int16_t _GyrOffset[3] = {0, 0, 0};

	///\brief Full scale resolution factors
	float _accelScale = 0.0f;
	float _gyroScale  = 0.0f;

	///\brief Full scale selections
	AccelFS _accelFS = gpm16;
	GyroFS  _gyroFS  = dps2000;

	///\brief Accel calibration
	float _accBD[3]  = {};
	float _accB[3]   = {};
	float _accS[3]   = {1.0f, 1.0f, 1.0f};
	float _accMax[3] = {};
	float _accMin[3] = {};

	///\brief Gyro calibration
	float _gyroBD[3] = {};
	float _gyrB[3]   = {};

	struct filter_t _angle = {0.0f, 0.0f};  ///< angle from filter
	HAL::FilterType _filter_type = HAL::FilterType::NONE;  ///< filter type
	Kalman _kalman;  ///< Kalman filter for angle estimation
	float _pitch_kalman = 0.0f;  ///< angle from Kalman filter
	float _pitch_comp = 0.0f;  ///< angle from complementary filter
	float _pitch_gyro = 0.0f;  ///< angle from gyro integration
	float _pitch_acc = 0.0f;  ///< angle from accelerometer

	float gyroXFV = 0.0f;  ///< low pass filter for gyro X axis

	///\brief Constants
	static constexpr uint8_t WHO_AM_I          = 0x47;  ///< expected value in UB0_REG_WHO_AM_I reg
	static constexpr int     NUM_CALIB_SAMPLES = 1000;  ///< for gyro/accel bias calib

	///\brief Conversion formula to get temperature in Celsius (Sec 4.13)
	static constexpr float TEMP_DATA_REG_SCALE = 132.48f;
	static constexpr float TEMP_OFFSET         = 25.0f;

	uint8_t _bank = 0;  ///< current user bank

	const uint8_t FIFO_EN      = 0x5F;
	const uint8_t FIFO_TEMP_EN = 0x04;
	const uint8_t FIFO_GYRO    = 0x02;
	const uint8_t FIFO_ACCEL   = 0x01;
	// const uint8_t FIFO_COUNT = 0x2E;
	// const uint8_t FIFO_DATA = 0x30;

	// BANK 1
	// const uint8_t GYRO_CONFIG_STATIC2 = 0x0B;
	const uint8_t GYRO_NF_ENABLE   = 0x00;
	const uint8_t GYRO_NF_DISABLE  = 0x01;
	const uint8_t GYRO_AAF_ENABLE  = 0x00;
	const uint8_t GYRO_AAF_DISABLE = 0x02;

	// BANK 2
	// const uint8_t ACCEL_CONFIG_STATIC2 = 0x03;
	const uint8_t ACCEL_AAF_ENABLE  = 0x00;
	const uint8_t ACCEL_AAF_DISABLE = 0x01;

	// private functions
	int writeRegister(uint8_t subAddress, uint8_t data);
	int readRegisters(uint8_t subAddress, uint8_t count, uint8_t* dest);
	int setBank(uint8_t bank);

	/**
     * @brief      Software reset of the device
     */
	void reset();

	/**
     * @brief      Read the WHO_AM_I register
     *
     * @return     Value of WHO_AM_I register
     */
	uint8_t whoAmI();

	void selfTest(int16_t* accelDiff, int16_t* gyroDiff, float* ratio);
};

class ICM42688_FIFO: public ICM42688 {
 public:
	using ICM42688::ICM42688;
	int  enableFifo(bool accel, bool gyro, bool temp);
	int  streamToFifo();
	int  readFifo();
	void getFifoAccelX_mss(size_t* size, float* data);
	void getFifoAccelY_mss(size_t* size, float* data);
	void getFifoAccelZ_mss(size_t* size, float* data);
	void getFifoGyroX(size_t* size, float* data);
	void getFifoGyroY(size_t* size, float* data);
	void getFifoGyroZ(size_t* size, float* data);
	void getFifoTemperature_C(size_t* size, float* data);

 protected:
	// fifo
	bool   _enFifoAccel     = false;
	bool   _enFifoGyro      = false;
	bool   _enFifoTemp      = false;
	bool   _enFifoTimestamp = false;
	bool   _enFifoHeader    = false;
	size_t _fifoSize        = 0;
	size_t _fifoFrameSize   = 0;
	float  _axFifo[85]      = {};
	float  _ayFifo[85]      = {};
	float  _azFifo[85]      = {};
	size_t _aSize           = 0;
	float  _gxFifo[85]      = {};
	float  _gyFifo[85]      = {};
	float  _gzFifo[85]      = {};
	size_t _gSize           = 0;
	float  _tFifo[256]      = {};
	size_t _tSize           = 0;
};

namespace ICM42688reg {

// Accesible from all user banks
static constexpr uint8_t REG_BANK_SEL = 0x76;

// User Bank 0
static constexpr uint8_t UB0_REG_DEVICE_CONFIG = 0x11;
// break
static constexpr uint8_t UB0_REG_DRIVE_CONFIG = 0x13;
static constexpr uint8_t UB0_REG_INT_CONFIG   = 0x14;
// break
static constexpr uint8_t UB0_REG_FIFO_CONFIG = 0x16;
// break
static constexpr uint8_t UB0_REG_TEMP_DATA1    = 0x1D;
static constexpr uint8_t UB0_REG_TEMP_DATA0    = 0x1E;
static constexpr uint8_t UB0_REG_ACCEL_DATA_X1 = 0x1F;
static constexpr uint8_t UB0_REG_ACCEL_DATA_X0 = 0x20;
static constexpr uint8_t UB0_REG_ACCEL_DATA_Y1 = 0x21;
static constexpr uint8_t UB0_REG_ACCEL_DATA_Y0 = 0x22;
static constexpr uint8_t UB0_REG_ACCEL_DATA_Z1 = 0x23;
static constexpr uint8_t UB0_REG_ACCEL_DATA_Z0 = 0x24;
static constexpr uint8_t UB0_REG_GYRO_DATA_X1  = 0x25;
static constexpr uint8_t UB0_REG_GYRO_DATA_X0  = 0x26;
static constexpr uint8_t UB0_REG_GYRO_DATA_Y1  = 0x27;
static constexpr uint8_t UB0_REG_GYRO_DATA_Y0  = 0x28;
static constexpr uint8_t UB0_REG_GYRO_DATA_Z1  = 0x29;
static constexpr uint8_t UB0_REG_GYRO_DATA_Z0  = 0x2A;
static constexpr uint8_t UB0_REG_TMST_FSYNCH   = 0x2B;
static constexpr uint8_t UB0_REG_TMST_FSYNCL   = 0x2C;
static constexpr uint8_t UB0_REG_INT_STATUS    = 0x2D;
static constexpr uint8_t UB0_REG_FIFO_COUNTH   = 0x2E;
static constexpr uint8_t UB0_REG_FIFO_COUNTL   = 0x2F;
static constexpr uint8_t UB0_REG_FIFO_DATA     = 0x30;
static constexpr uint8_t UB0_REG_APEX_DATA0    = 0x31;
static constexpr uint8_t UB0_REG_APEX_DATA1    = 0x32;
static constexpr uint8_t UB0_REG_APEX_DATA2    = 0x33;
static constexpr uint8_t UB0_REG_APEX_DATA3    = 0x34;
static constexpr uint8_t UB0_REG_APEX_DATA4    = 0x35;
static constexpr uint8_t UB0_REG_APEX_DATA5    = 0x36;
static constexpr uint8_t UB0_REG_INT_STATUS2   = 0x37;
static constexpr uint8_t UB0_REG_INT_STATUS3   = 0x38;
// break
static constexpr uint8_t UB0_REG_SIGNAL_PATH_RESET  = 0x4B;
static constexpr uint8_t UB0_REG_INTF_CONFIG0       = 0x4C;
static constexpr uint8_t UB0_REG_INTF_CONFIG1       = 0x4D;
static constexpr uint8_t UB0_REG_PWR_MGMT0          = 0x4E;
static constexpr uint8_t UB0_REG_GYRO_CONFIG0       = 0x4F;
static constexpr uint8_t UB0_REG_ACCEL_CONFIG0      = 0x50;
static constexpr uint8_t UB0_REG_GYRO_CONFIG1       = 0x51;
static constexpr uint8_t UB0_REG_GYRO_ACCEL_CONFIG0 = 0x52;
static constexpr uint8_t UB0_REG_ACCEFL_CONFIG1     = 0x53;
static constexpr uint8_t UB0_REG_TMST_CONFIG        = 0x54;
// break
static constexpr uint8_t UB0_REG_APEX_CONFIG0 = 0x56;
static constexpr uint8_t UB0_REG_SMD_CONFIG   = 0x57;
// break
static constexpr uint8_t UB0_REG_FIFO_CONFIG1 = 0x5F;
static constexpr uint8_t UB0_REG_FIFO_CONFIG2 = 0x60;
static constexpr uint8_t UB0_REG_FIFO_CONFIG3 = 0x61;
static constexpr uint8_t UB0_REG_FSYNC_CONFIG = 0x62;
static constexpr uint8_t UB0_REG_INT_CONFIG0  = 0x63;
static constexpr uint8_t UB0_REG_INT_CONFIG1  = 0x64;
static constexpr uint8_t UB0_REG_INT_SOURCE0  = 0x65;
static constexpr uint8_t UB0_REG_INT_SOURCE1  = 0x66;
// break
static constexpr uint8_t UB0_REG_INT_SOURCE3 = 0x68;
static constexpr uint8_t UB0_REG_INT_SOURCE4 = 0x69;
// break
static constexpr uint8_t UB0_REG_FIFO_LOST_PKT0 = 0x6C;
static constexpr uint8_t UB0_REG_FIFO_LOST_PKT1 = 0x6D;
// break
static constexpr uint8_t UB0_REG_SELF_TEST_CONFIG = 0x70;
// break
static constexpr uint8_t UB0_REG_WHO_AM_I = 0x75;

// User Bank 1
static constexpr uint8_t UB1_REG_SENSOR_CONFIG0 = 0x03;
// break
static constexpr uint8_t UB1_REG_GYRO_CONFIG_STATIC2  = 0x0B;
static constexpr uint8_t UB1_REG_GYRO_CONFIG_STATIC3  = 0x0C;
static constexpr uint8_t UB1_REG_GYRO_CONFIG_STATIC4  = 0x0D;
static constexpr uint8_t UB1_REG_GYRO_CONFIG_STATIC5  = 0x0E;
static constexpr uint8_t UB1_REG_GYRO_CONFIG_STATIC6  = 0x0F;
static constexpr uint8_t UB1_REG_GYRO_CONFIG_STATIC7  = 0x10;
static constexpr uint8_t UB1_REG_GYRO_CONFIG_STATIC8  = 0x11;
static constexpr uint8_t UB1_REG_GYRO_CONFIG_STATIC9  = 0x12;
static constexpr uint8_t UB1_REG_GYRO_CONFIG_STATIC10 = 0x13;
// break
static constexpr uint8_t UB1_REG_XG_ST_DATA = 0x5F;
static constexpr uint8_t UB1_REG_YG_ST_DATA = 0x60;
static constexpr uint8_t UB1_REG_ZG_ST_DATA = 0x61;
static constexpr uint8_t UB1_REG_TMSTVAL0   = 0x62;
static constexpr uint8_t UB1_REG_TMSTVAL1   = 0x63;
static constexpr uint8_t UB1_REG_TMSTVAL2   = 0x64;
// break
static constexpr uint8_t UB1_REG_INTF_CONFIG4 = 0x7A;
static constexpr uint8_t UB1_REG_INTF_CONFIG5 = 0x7B;
static constexpr uint8_t UB1_REG_INTF_CONFIG6 = 0x7C;

// User Bank 2
static constexpr uint8_t UB2_REG_ACCEL_CONFIG_STATIC2 = 0x03;
static constexpr uint8_t UB2_REG_ACCEL_CONFIG_STATIC3 = 0x04;
static constexpr uint8_t UB2_REG_ACCEL_CONFIG_STATIC4 = 0x05;
// break
static constexpr uint8_t UB2_REG_XA_ST_DATA = 0x3B;
static constexpr uint8_t UB2_REG_YA_ST_DATA = 0x3C;
static constexpr uint8_t UB2_REG_ZA_ST_DATA = 0x3D;

// User Bank 4
static constexpr uint8_t UB4_REG_APEX_CONFIG1 = 0x40;
static constexpr uint8_t UB4_REG_APEX_CONFIG2 = 0x41;
static constexpr uint8_t UB4_REG_APEX_CONFIG3 = 0x42;
static constexpr uint8_t UB4_REG_APEX_CONFIG4 = 0x43;
static constexpr uint8_t UB4_REG_APEX_CONFIG5 = 0x44;
static constexpr uint8_t UB4_REG_APEX_CONFIG6 = 0x45;
static constexpr uint8_t UB4_REG_APEX_CONFIG7 = 0x46;
static constexpr uint8_t UB4_REG_APEX_CONFIG8 = 0x47;
static constexpr uint8_t UB4_REG_APEX_CONFIG9 = 0x48;
// break
static constexpr uint8_t UB4_REG_ACCEL_WOM_X_THR = 0x4A;
static constexpr uint8_t UB4_REG_ACCEL_WOM_Y_THR = 0x4B;
static constexpr uint8_t UB4_REG_ACCEL_WOM_Z_THR = 0x4C;
static constexpr uint8_t UB4_REG_INT_SOURCE6     = 0x4D;
static constexpr uint8_t UB4_REG_INT_SOURCE7     = 0x4E;
static constexpr uint8_t UB4_REG_INT_SOURCE8     = 0x4F;
static constexpr uint8_t UB4_REG_INT_SOURCE9     = 0x50;
static constexpr uint8_t UB4_REG_INT_SOURCE10    = 0x51;
// break
static constexpr uint8_t UB4_REG_OFFSET_USER0 = 0x77;
static constexpr uint8_t UB4_REG_OFFSET_USER1 = 0x78;
static constexpr uint8_t UB4_REG_OFFSET_USER2 = 0x79;
static constexpr uint8_t UB4_REG_OFFSET_USER3 = 0x7A;
static constexpr uint8_t UB4_REG_OFFSET_USER4 = 0x7B;
static constexpr uint8_t UB4_REG_OFFSET_USER5 = 0x7C;
static constexpr uint8_t UB4_REG_OFFSET_USER6 = 0x7D;
static constexpr uint8_t UB4_REG_OFFSET_USER7 = 0x7E;
static constexpr uint8_t UB4_REG_OFFSET_USER8 = 0x7F;

}  // namespace ICM42688reg

#endif  // ICM42688_H
