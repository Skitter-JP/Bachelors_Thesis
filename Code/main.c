#include "sam.h"
#include "pin_macros.h"
#include "pinout.h"
#include "USB.h"
#include <string.h>
#include <math.h>
#include "tgmath.h"

typedef struct PID_struct{
	
	float P; //!< Proportional gain
	float _I; //!< Integral gain
	float D; //!< Derivative gain
	float output_ramp; //!< Maximum speed of change of the output value
	float limit; //!< Maximum output value
	float integral_prev; //!< last integral component value
	float error_prev; //!< last tracking error value
	unsigned long timestamp_prev; //!< Last execution timestamp
	float output_prev;  //!< last pid output value
	
}PID_struct;

typedef struct LPF_struct{
	
	volatile float Tf; //!< Low pass filter time constant
	volatile unsigned long timestamp_prev;  //!< Last execution timestamp
	volatile float y_prev; //!< filtered value in previous execution step
	
}LPF_struct;

typedef struct PWM_struct{

	float voltage_power_supply;
	float voltage_limit;

	float PWM_resolution;

	int modulation_type;

	volatile float dc_a;
	volatile float dc_b;
	volatile float dc_c;

}PWM_struct;

typedef struct OBSERVER_struct{

	unsigned long timestamp_prev; 
	unsigned long timestamp_now; 

	float flux_linkage;
	float observer_gain;	
	
	float x2;
	float x1;
	
	float flux_vector_alpha;
	float flux_vector_beta;
	
	volatile float xa;
	
	volatile float xb;

}OBSERVER_struct;

typedef struct MOTOR_struct{
	
	_Bool sensorless;

	int pole_pairs;
	int motor_state;

	float phase_resistance;
	float inductance;
	float flux_linkage;
	float observer_gain;

	float current_limit;

	float current_sp;
	
	volatile float exact_shaft_angle;
	
	volatile float shaft_angle;
	
	volatile float shaft_velocity;

	volatile float estimated_angle;
	
	volatile float measured_angle;

	volatile float electrical_angle;

	
	volatile float U_phase_A;
	volatile float U_phase_B;
	volatile float U_phase_C;
	
	volatile float U_q;
	volatile float U_d;
	
	volatile float U_q_prime;
	volatile float U_d_prime;
	
	volatile float U_alpha;
	volatile float U_beta;
	
	volatile float U_alpha_prime;
	volatile float U_beta_prime;
	
	volatile float I_phase_A;
	volatile float I_phase_B;
	volatile float I_phase_C;
	
	volatile float I_q;
	volatile float I_d;
	
	volatile float I_alpha;
	volatile float I_beta;
	
	
	
}MOTOR_struct;

typedef struct CURRENT_SENSE_struct{
	
	

	float shunt_resistor;
	float reference_voltage;
	float gain;
	float bit_rate;

	float volts_to_amps_ratio;

	float I_phase_A_offset;
	float I_phase_B_offset;
	float I_phase_C_offset;

	float I_gain_phase_A;
	float I_gain_phase_B;
	float I_gain_phase_C;

	float conversion_scalar;
	
	volatile uint32_t I_raw_phase_A;
	volatile uint32_t I_raw_phase_B;
	volatile uint32_t I_raw_phase_C;
	
	volatile float I_raw_phase_A_voltage;
	volatile float I_raw_phase_B_voltage;
	volatile float I_raw_phase_C_voltage;
	
	volatile _Bool read_currents;


}CURRENT_SENSE_struct;

enum Sensor_Direction{
	CLOCK_WISE = 1,
	COUNTER_CLOCK_WISE = -1,
	NONE
};

enum modulation{
	SINE_PWM,
	SPVM
};

enum calcuations{
	APROXIMATE,
	COMPUTE
};

enum states{
	DISABLED = 0,
	ENABLED = 1,
	UNKNOWN = 2
};

enum phases{

	PHASE_A = 1,
	PHASE_B = 2,
	PHASE_C = 3
	
};

enum bools{
	FALSE = 0,
	TRUE = 1
};

enum motions{
	ANGLE = 0,
	ANGLE_OPENLOOP = 1,
	VELOCITY = 2,
	VELOCITY_OPENLOOP = 3,
	TORQUE = 4
};

#define _NOP() do { __asm__ __volatile__ ("nop"); } while (0)

#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define bitSet(value, bit) ((value) |= (1UL << (bit)))
#define bitClear(value, bit) ((value) &= ~(1UL << (bit)))
#define bitWrite(value, bit, bitvalue) (bitvalue ? bitSet(value, bit) : bitClear(value, bit))
#define bit(b) (1UL << (b))


//#define CPU_CLK (120000000ul) 
#define F_CPU	(120000000ul) //120 MHz

#define SERCOM_SPI_FREQ_REF 48000000 // 48MHz

//NaN and Infinity checks
#define UTILS_IS_INF(x)		((x) == (1.0 / 0.0) || (x) == (-1.0 / 0.0))
#define UTILS_IS_NAN(x)		((x) != (x))
#define UTILS_NAN_ZERO(x)	(x = UTILS_IS_NAN(x) ? 0.0 : x)

// Value for PCHCTRLm Mapping
#define GCLK_TCC0 25
#define GCLK_TCC2 29
#define GCLK_TC0 9

#define PORT_MUX_B 1
#define PORT_MUX_D 3
#define PORT_MUX_F 5
#define PORT_MUX_G 6
#define PORT_MUX_H 7

//For Clock Setup
#define GENERIC_CLOCK_GENERATOR_MAIN      (0u)
#define GENERIC_CLOCK_GENERATOR_XOSC32K   (3u)
#define GENERIC_CLOCK_GENERATOR_48M		  (1u)
#define GENERIC_CLOCK_GENERATOR_48M_SYNC	GCLK_SYNCBUSY_GENCTRL1
#define GENERIC_CLOCK_GENERATOR_100M	  (2u)
#define GENERIC_CLOCK_GENERATOR_100M_SYNC	GCLK_SYNCBUSY_GENCTRL2
#define GENERIC_CLOCK_GENERATOR_12M       (4u)
#define GENERIC_CLOCK_GENERATOR_12M_SYNC   GCLK_SYNCBUSY_GENCTRL4
#define MAIN_CLOCK_SOURCE				  GCLK_GENCTRL_SRC_DPLL0
#define GENERIC_CLOCK_GENERATOR_1M		  (5u)
//For Clock Setup - END


#define _sign(a) ( ( (a) < 0 )  ?  -1   : ( (a) > 0 ) )
#define _round(x) ((x)>=0?(long)((x)+0.5):(long)((x)-0.5))
#define _constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define min(a,b) ((a)<(b)?(a):(b))
#define max(a,b) ((a)>(b)?(a):(b))
#define _sqrt(a) (_sqrtApprox(a))

// utility defines
#define _2_SQRT3 1.15470053838
#define _SQRT3 1.73205080757
#define _1_SQRT3 0.57735026919
#define _SQRT3_2 0.86602540378
#define _SQRT2 1.41421356237
#define _120_D2R 2.09439510239
#define _PI 3.14159265359
#define _PI_2 1.57079632679
#define _PI_3 1.0471975512
#define _2PI 6.28318530718
#define _3PI_2 4.71238898038
#define _PI_6 0.52359877559
#define _2PI_3 2.094395102 //120 deg
#define _4PI_3 4.188790205 //240 deg
#define _SQRT3_2 0.86602540378


#define velocity_limit 20 //[rad/sec]

//This will limit the amount of call to the ADC and observer ?
#define FOC_CONTROL_LOOP_FREQ_DIVIDER	3


//Command for the Magnetic Sensor, can be defined ...
const uint16_t AS5048A_NOP                           = 0x0000; // Dummy operation, no information.
const uint16_t AS5048A_CLEAR_ERROR_FLAG              = 0x0001; // Error register. All errors are cleared by access.
const uint16_t AS5048A_PROGRAMMING_CONTROL           = 0x0003; // Register control programming. Programming must be enabled before the memory is burned. Before programming verification is mandatory. See Programming Procedure.
const uint16_t AS5048A_OTP_REGISTER_ZERO_POS_HIGH    = 0x0016; // Zero value of 8 bits high
const uint16_t AS5048A_OTP_REGISTER_ZERO_POS_LOW     = 0x0017; // Zero value 6 bits lower
const uint16_t AS5048A_DIAG_AGC                      = 0x3FFD; // (0-7) The value of the automatic gain control. 0 decimal represents a high magnetic field, 255 decimal represents a low magnetic field. (8-13) Diagnostics Flags
const uint16_t AS5048A_MAGNITUDE                     = 0x3FFE; // The value of the output power CORDIC
const uint16_t AS5048A_ANGLE                         = 0x3FFF; // Angular output value, including zero position correction Resolution_ADC 14-bit resolution (0.0219 ° / LSB)



static volatile uint32_t microseconds = 0;
unsigned long open_loop_timestamp = 0;
unsigned long velocity_calc_timestamp = 0;
const int sine_array[200] = {0,79,158,237,316,395,473,552,631,710,789,867,946,1024,1103,1181,1260,1338,1416,1494,1572,1650,1728,1806,1883,1961,2038,2115,2192,2269,2346,2423,2499,2575,2652,2728,2804,2879,2955,3030,3105,3180,3255,3329,3404,3478,3552,3625,3699,3772,3845,3918,3990,4063,4135,4206,4278,4349,4420,4491,4561,4631,4701,4770,4840,4909,4977,5046,5113,5181,5249,5316,5382,5449,5515,5580,5646,5711,5775,5839,5903,5967,6030,6093,6155,6217,6279,6340,6401,6461,6521,6581,6640,6699,6758,6815,6873,6930,6987,7043,7099,7154,7209,7264,7318,7371,7424,7477,7529,7581,7632,7683,7733,7783,7832,7881,7930,7977,8025,8072,8118,8164,8209,8254,8298,8342,8385,8428,8470,8512,8553,8594,8634,8673,8712,8751,8789,8826,8863,8899,8935,8970,9005,9039,9072,9105,9138,9169,9201,9231,9261,9291,9320,9348,9376,9403,9429,9455,9481,9506,9530,9554,9577,9599,9621,9642,9663,9683,9702,9721,9739,9757,9774,9790,9806,9821,9836,9850,9863,9876,9888,9899,9910,9920,9930,9939,9947,9955,9962,9969,9975,9980,9985,9989,9992,9995,9997,9999,10000,10000};


static volatile uint32_t _ulTickCount=0 ;



static volatile float angle_prev =0;

static int angle_data_prev = 0;
float full_rotation_offset = 0.0; //!<number of full rotations made


unsigned long previousMillis =0;
long interval = 1000;

volatile uint32_t pwm_min = 10000;
volatile uint32_t pwm_max = 0;


uint8_t rx_data_buffer[20]; // buffer for data reception
uint8_t tmp_buffer[20]; // temporary buffer



_Bool error_flag = 0;


float FOC_target = 0;


float shaft_angle_sp =0;
float shaft_velocity_sp = 0;

static _Bool aprox_trig = FALSE;

// 1 -> Clock Wise
// -1 -> Counter Clock Wise
int sensor_direction = NONE;


// What ?
float zero_electric_angle = 0;


// BEGIN - Current Sensing Variables
_Bool current_sense = DISABLED;
// END - Current Sensing Variables

//The type of motion the motor will perform
int motion_control_type = ANGLE_OPENLOOP;



//Objects for PID Control
PID_struct PID_velocity_obj;
PID_struct PID_angle_obj;
PID_struct PID_Iq_obj;
PID_struct PID_Id_obj;

//Objects for Low Pass Filter
LPF_struct LPF_velocity_obj;
LPF_struct LPF_angle_obj;
LPF_struct LPF_Iq_obj;
LPF_struct LPF_Id_obj;

//Motor Object
MOTOR_struct MOTOR_obj;

//Current Sense Object
CURRENT_SENSE_struct CURRENT_obj;

//PWM Object
PWM_struct PWM_obj;

//Observer Object
OBSERVER_struct OBSERVER_obj;


float buffer[64];
uint8_t buffer_pos=0;
uint8_t buffer_pos_1=0;

int enabale_ob = FALSE;

void observer();
void set_up_PWM_obj();
void set_up_motor_obj();
void set_up_current_obj();
void get_currents(float electrical_angle);
void FOC(float target);
float shaftVelocity(float _current_angle);
float shaftAngle();
void modulate(float Uq, float Ud, float angle_el);
void angular_sensor_calibrate();
float getAngle();
void set_up_LPF();
void set_up_PID();
float LPF(float value, struct LPF_struct *LPF_obj);
float PID(float err, struct PID_struct *PID_obj);
float _sqrtApprox(float number);
unsigned long millis( void );
void delay( unsigned long ms );
float getVelocity(float _current_angle);
void velocity_OL(float target_velocity);
float electricalAngle(float _angle);
void velocity_CL(float target_velocity);
void Angle_CL(float target_angle);
void Angle_OL(float target_velocity);
void SinePWM(float Uq, float Ud, float angle_el);
void SpaceVectorPWM(float Uq, float Ud, float angle_el);
void delayMicroseconds(unsigned int us);
unsigned long micros( void );
void setPhaseVoltage(float Uphase_1, float Uphase_2, float Uphase_3);
float _cos(float a);
float _sin(float a);
float _normalizeAngle(float angle);
float _electricalAngle(float shaft_angle, int pole_pairs) ;
void TC0_Handler();
void TCC0_0_Handler();
void TCC0_setup();
void set_TC0();
void PINs_setup();
uint16_t AS5048A_send_command(int cmd);
uint16_t AS5048A_read_angle();
char CalcEvenParity(uint16_t Value);
uint8_t  SPI_transfer_8_bits(uint8_t data);
uint16_t SPI_transfer_16_bits(uint16_t data);
void set_up_SPI_SERCOM_5(uint32_t baudrate);
void set_up_clocks();
void voltage_ref_setup();
void set_up_ADC0();
uint32_t ADC_Read(int phase);
void usb_data_received(uint8_t status, uint16_t length) ;
void set_usb_config(uint8_t index) ;
void set_up_systick() ;
void SysTick_Handler();
void dwt_enable(void);

void observer(){
	
	OBSERVER_obj.timestamp_now = micros();
	
	float xa_d, xb_d;
	float err;
	float theta;
	
	const float L = (3.0 / 2.0) *MOTOR_obj.inductance;
	float R = (3.0 / 2.0) *MOTOR_obj.phase_resistance;
	
	/*
	const float L = MOTOR_obj.inductance;
	float R = MOTOR_obj.phase_resistance;
	*/
	//Inductive Components
	const float LI_A = MOTOR_obj.I_alpha*L;
	const float LI_B = MOTOR_obj.I_beta*L; 
	//Resistive Components
	const float RI_A = MOTOR_obj.I_alpha*R;
	const float RI_B = MOTOR_obj.I_beta*R;
	const float FL_2 = pow(MOTOR_obj.flux_linkage,2);

	const float gain_half = MOTOR_obj.observer_gain;;

	
	
	
	err = FL_2 - (pow((OBSERVER_obj.xa-LI_A),2)+pow((OBSERVER_obj.xb-LI_B),2));
	
	xa_d = -RI_A + MOTOR_obj.U_alpha + gain_half*(OBSERVER_obj.xa- LI_A)*err;
	xb_d = -RI_B + MOTOR_obj.U_beta + gain_half*(OBSERVER_obj.xb- LI_B)*err;

	float dt = (OBSERVER_obj.timestamp_now - OBSERVER_obj.timestamp_prev)*1e-6;



	OBSERVER_obj.xa += (xa_d * dt);
	OBSERVER_obj.xb += (xb_d * dt);
	

	
	UTILS_NAN_ZERO(OBSERVER_obj.xa);
	UTILS_NAN_ZERO(OBSERVER_obj.xb);
	
	theta = atan2(OBSERVER_obj.xb - LI_B, OBSERVER_obj.xa - LI_A);
	
	OBSERVER_obj.timestamp_prev = OBSERVER_obj.timestamp_now;
	
	MOTOR_obj.estimated_angle = theta+3;
	
	buffer[buffer_pos++]= MOTOR_obj.estimated_angle;
	buffer[buffer_pos+15] = MOTOR_obj.electrical_angle;
	buffer[buffer_pos+31]= OBSERVER_obj.xa;
	buffer[buffer_pos+47]= OBSERVER_obj.xb;
	if (buffer_pos==(sizeof(buffer)/sizeof(buffer[0]))/4){
		buffer_pos=0;
		usb_send_bulk_data((uint8_t*)buffer,sizeof(buffer),NULL);
	}
	

}

void set_up_observer_obj(){
	
	
	//Setting the Flux Linkage
	//OBSERVER_obj.flux_linkage = 0.0024; //[Wb]
	
	OBSERVER_obj.flux_linkage = 0.004; //[Wb]
	
	//Setting the observers gain
	OBSERVER_obj.observer_gain = 4000;
	
	OBSERVER_obj.xa = 0;
	OBSERVER_obj.xb = 0;
	
	OBSERVER_obj.timestamp_prev= micros();
	OBSERVER_obj.timestamp_now = micros();
	
	
}

void set_up_PWM_obj(){

	//Set up the PWM Resolution
	PWM_obj.PWM_resolution = 1000;

	//Setup the Type of Modulation -> Sineusoid Modulation by default
	PWM_obj.modulation_type = SINE_PWM;

	//Set the power supply voltage
	PWM_obj.voltage_power_supply = 12.0;

	//Set the voltage limit to the motor
	PWM_obj.voltage_limit = 2.0;
}

void set_up_motor_obj(){

	//Is the motors angle estimated by current ?
	MOTOR_obj.sensorless = FALSE;

	//Motor State Enable
	MOTOR_obj.motor_state = ENABLED;

	//We Set the phase resistance of the motor
	MOTOR_obj.phase_resistance = 1.6; //[Ohms]

	//Set up the number of pole pairs
	MOTOR_obj.pole_pairs = 7;

	//Set inductance	//We need to measure it...
	MOTOR_obj.inductance = 0.00002;
	
	

	//Setting the Current Limit
	MOTOR_obj.current_limit = 1.0; // [Amps]
	

	


}

void set_up_current_obj(){

	//Shunt Resitor Value ... 3mOhm
	CURRENT_obj.shunt_resistor = 0.003;

	//Bit rate
	//For 10 bits
	CURRENT_obj.bit_rate = pow(2,10);
	//For 12 bits
	//CURRENT_obj.bit_rate = pow(2,12);

	//gain of OP Amps
	//gain of the Amps are set to 20
	CURRENT_obj.gain = 20;

	//Reference Voltage of ADC
	//Refence Voltage is set to 3.3V
	CURRENT_obj.reference_voltage = 3.3;
	//CURRENT_obj.reference_voltage = 1.0;

	//Calculate Conversion
	//Using the following eqaution = V_measure = ADC_value*(ADC_V_ref/ADC_bit_rate)
	CURRENT_obj.conversion_scalar = CURRENT_obj.reference_voltage/CURRENT_obj.bit_rate;

	//Set offsets to Zero until they are calculated
	CURRENT_obj.I_phase_A_offset = 0;
	CURRENT_obj.I_phase_B_offset = 0;
 	CURRENT_obj.I_phase_C_offset = 0;
	
	//Calculating Volt to Amp Ratio --> Multiplying the Voltage by this will give us the current
	CURRENT_obj.volts_to_amps_ratio = 1.0 /CURRENT_obj.shunt_resistor / CURRENT_obj.gain; 

	//Saving the V/I ratio for each phase.
	CURRENT_obj.I_gain_phase_A = CURRENT_obj.volts_to_amps_ratio;
	CURRENT_obj.I_gain_phase_B = CURRENT_obj.volts_to_amps_ratio;
	CURRENT_obj.I_gain_phase_C = CURRENT_obj.volts_to_amps_ratio;

	const int calibration_rounds = 1000;

	//Here we will calculate the average offset off phase	
	
	for (int i = 0; i < calibration_rounds; i++) {
		volatile uint32_t p_a = ADC_Read(PHASE_A);
		CURRENT_obj.I_phase_A_offset += p_a*CURRENT_obj.conversion_scalar;
		CURRENT_obj.I_phase_B_offset += ADC_Read(PHASE_B)*CURRENT_obj.conversion_scalar;
		CURRENT_obj.I_phase_C_offset += ADC_Read(PHASE_C)*CURRENT_obj.conversion_scalar;
		delay(1);
	}

	//We calculate the mean offsets 
	CURRENT_obj.I_phase_A_offset = CURRENT_obj.I_phase_A_offset / calibration_rounds;
	CURRENT_obj.I_phase_B_offset = CURRENT_obj.I_phase_B_offset / calibration_rounds;
	CURRENT_obj.I_phase_C_offset = CURRENT_obj.I_phase_C_offset / calibration_rounds;

	//Offset should now be calibrated.
}

void get_FOC_currents(float electrical_angle){
	
	/*
	CURRENT_obj.read_currents = TRUE;
	
	//Wait for currents to be read by TCC0 interupt
	while(CURRENT_obj.read_currents);

	//Currents have been read, we now convert them from RAW form to Voltage form
	CURRENT_obj.I_raw_phase_A_voltage = CURRENT_obj.I_raw_phase_A*CURRENT_obj.conversion_scalar;
	CURRENT_obj.I_raw_phase_B_voltage = CURRENT_obj.I_raw_phase_B*CURRENT_obj.conversion_scalar;
	CURRENT_obj.I_raw_phase_C_voltage = CURRENT_obj.I_raw_phase_C*CURRENT_obj.conversion_scalar;
	
	//CURRENT_obj.I_raw_phase_C_voltage = CURRENT_obj.I_raw_phase_A_voltage + CURRENT_obj.I_raw_phase_B_voltage;

	//We now subtract the offset of the current sensing and apply the Voltage to Current conversion
	MOTOR_obj.I_phase_A = (CURRENT_obj.I_raw_phase_A_voltage - CURRENT_obj.I_phase_A_offset)*CURRENT_obj.I_gain_phase_A;// amps
	MOTOR_obj.I_phase_B = (CURRENT_obj.I_raw_phase_B_voltage - CURRENT_obj.I_phase_B_offset)*CURRENT_obj.I_gain_phase_B;// amps
	MOTOR_obj.I_phase_C = (CURRENT_obj.I_raw_phase_C_voltage - CURRENT_obj.I_phase_C_offset)*CURRENT_obj.I_gain_phase_C;// amps

	//We Calculate the current in Phase C using KCL
    //MOTOR_obj.I_phase_C = MOTOR_obj.I_phase_A + MOTOR_obj.I_phase_B ;

	/*

	// signal filtering using identity a + b + c = 0. Assumes measurement error is normally distributed.	
	float mid = (1.f/3) * (MOTOR_obj.I_phase_A + MOTOR_obj.I_phase_B  + MOTOR_obj.I_phase_C );
	float a = MOTOR_obj.I_phase_A - mid;
	float b = MOTOR_obj.I_phase_B - mid;
	//Calculate Clarke Transform
	MOTOR_obj.I_alpha = a;
	MOTOR_obj.I_beta = _1_SQRT3 * a + _2_SQRT3 * b;
	*/
	
	/*
	MOTOR_obj.I_alpha = (2.f/3)*MOTOR_obj.I_phase_A - (1.f/3)*(MOTOR_obj.I_phase_B-MOTOR_obj.I_phase_C);
	MOTOR_obj.I_beta = _2_SQRT3*(MOTOR_obj.I_phase_B - MOTOR_obj.I_phase_C);
	*/
	
	MOTOR_obj.I_alpha = MOTOR_obj.I_phase_A;
	MOTOR_obj.I_beta = _1_SQRT3*MOTOR_obj.I_phase_A + _2_SQRT3*MOTOR_obj.I_phase_B;
	
	/*
	buffer[buffer_pos++]=MOTOR_obj.I_alpha;
	if (buffer_pos==sizeof(buffer)/sizeof(buffer[0])){
		buffer_pos=0;
		usb_send_bulk_data((uint8_t*)buffer,sizeof(buffer),NULL);
	}
	*/

	float ct;
	float st;
	
	if(aprox_trig){
		ct = _cos(electrical_angle);
		st = _sin(electrical_angle);
	}else{
		ct = cos(electrical_angle);
		st = sin(electrical_angle);
	}
	//Calculate Park Transform	
	MOTOR_obj.I_d = MOTOR_obj.I_alpha * ct + MOTOR_obj.I_beta * st;
	MOTOR_obj.I_q = MOTOR_obj.I_beta * ct - MOTOR_obj.I_alpha * st;
	
	
	MOTOR_obj.I_d = LPF(MOTOR_obj.I_d,&LPF_Id_obj);
	MOTOR_obj.I_q = LPF(MOTOR_obj.I_q,&LPF_Iq_obj);
	
}


 int skip_count = 10;

void FOC(float target){
	
	
	//If the motor is disabled we do nothing
	if(MOTOR_obj.motor_state == DISABLED){
		setPhaseVoltage(0,0,0);
		return;
	}
	
	skip_count++;
	
	if(enabale_ob == TRUE){
		observer();	
	}
	
	
	if(skip_count >10){
		skip_count=0;
		
	}



	

	
	/*	
	
	buffer[buffer_pos++]=  MOTOR_obj.estimated_angle;
	buffer[buffer_pos+31] = 0;
	if (buffer_pos==(sizeof(buffer)/sizeof(buffer[0]))/2){
		buffer_pos=0;
		usb_send_bulk_data((uint8_t*)buffer,sizeof(buffer),NULL);
	}
	
	

	*/
	
	switch (motion_control_type){
		
		case VELOCITY_OPENLOOP:
			velocity_OL(target);
			MOTOR_obj.shaft_velocity = target;	
			//get_FOC_currents(MOTOR_obj.electrical_angle);
			return;
			break;
			
		case ANGLE_OPENLOOP:
			Angle_OL(target);
			MOTOR_obj.shaft_angle = target;
			//get_FOC_currents(MOTOR_obj.electrical_angle);
			return;
			break;
		
		case VELOCITY:
			velocity_CL(target);
			
			break;
		
		case ANGLE:
			Angle_CL(target);
			//get_FOC_currents(MOTOR_obj.electrical_angle);
			break;

			
	}
	
	
	
	
	get_FOC_currents(MOTOR_obj.electrical_angle);

	
	if(current_sense == ENABLED){
		
		/*
		MOTOR_obj.I_q = LPF(MOTOR_obj.I_q, &LPF_Iq_obj);
		MOTOR_obj.I_d = LPF(MOTOR_obj.I_d, &LPF_Id_obj);
		*/	
		MOTOR_obj.U_q = PID(MOTOR_obj.current_sp - MOTOR_obj.I_q, &PID_Iq_obj);
		MOTOR_obj.U_d = PID(-MOTOR_obj.I_d, &PID_Id_obj);
		
	}
	
	
	modulate(MOTOR_obj.U_q, MOTOR_obj.U_d, MOTOR_obj.electrical_angle);

}

float shaftVelocity(float _current_angle) {
	// if no sensor linked return previous value ( for open loop )
	
	return sensor_direction*LPF(getVelocity(_current_angle),&LPF_velocity_obj);
}

float shaftAngle(){
	
	return sensor_direction*LPF(getAngle(), &LPF_angle_obj);
}

void modulate(float Uq, float Ud, float angle_el){
	
	switch(PWM_obj.modulation_type){
		case SINE_PWM:
			SinePWM(Uq, Ud, angle_el);
			break;
		case SPVM:
			SpaceVectorPWM(Uq, Ud, angle_el);
			break;	
	}
	
}

void angular_sensor_calibrate(){
	
	unsigned long d = 2;
	
	//float voltage_sensor_align = PWM_obj.voltage_limit;
	
	float voltage_sensor_align = 2;
	
	for (int i = 0; i <=500; i++ ) {
		float angle = _3PI_2 + _2PI * i / 500.0;
		modulate(voltage_sensor_align, 0,  angle);
		delay(d);
	}
	// take and angle in the middle
	float mid_angle = getAngle();
	
	// move one electrical revolution backwards
	for (int i = 500; i >=0; i-- ) {
		float angle = _3PI_2 + _2PI * i / 500.0 ;
		modulate(voltage_sensor_align, 0,  angle);
		delay(d);
	}
	float end_angle = getAngle();
	modulate(0, 0, 0);
	delay(200);
	
	if(mid_angle<end_angle){
		// Counter Clock Wise
		sensor_direction = COUNTER_CLOCK_WISE;
	}else{
		// Clock Wise
		sensor_direction = CLOCK_WISE;
	}
	
	modulate(voltage_sensor_align, 0,  _3PI_2);
	delay(700);
	zero_electric_angle = _normalizeAngle(_electricalAngle(sensor_direction*getAngle(), MOTOR_obj.pole_pairs));
	delay(20);
	modulate(0, 0, 0);
	delay(200);
	
}

float getAngle(){
  // raw data from the sensor
  int angle_data = AS5048A_send_command(AS5048A_ANGLE); 

	if(angle_data ==0){
		
		angle_data = AS5048A_send_command(AS5048A_ANGLE);

	}

  // tracking the number of rotations 
  // in order to expand angle range form [0,2PI] 
  // to basically infinity
  int d_angle = angle_data - angle_data_prev;
  // if overflow happened track it as full rotation
  if(abs(d_angle) > (0.95*16384) ){
	
	if(d_angle > 0){
		
		full_rotation_offset += -_2PI;
	}else{

		full_rotation_offset += _2PI;
	}
	
  }
  

	
  
  // save the current angle value for the next steps
  // in order to know if overflow happened
  angle_data_prev = angle_data;
  
  MOTOR_obj.exact_shaft_angle =  ( angle_data / (float)16384) * _2PI;



  // return the full angle 
  // (number of full rotations)*2PI + current sensor angle 
  return full_rotation_offset + ( angle_data / (float)16384) * _2PI;
}

void set_up_LPF(){
	
	//LPF Velocity
	LPF_velocity_obj.timestamp_prev = micros();
	LPF_velocity_obj.y_prev=0;
	LPF_velocity_obj.Tf = 0.005;
	
	//LPF Angle	
	//Might not need this since the time constant is zero ??
	LPF_angle_obj.timestamp_prev = micros();
	LPF_angle_obj.y_prev=0;
	LPF_angle_obj.Tf = 0.001;
	
	//LPF Iq	
	LPF_Iq_obj.timestamp_prev = micros();
	LPF_Iq_obj.y_prev=0;
	LPF_Iq_obj.Tf =  0.0005;
		
	//LPF Id
	LPF_Id_obj.timestamp_prev = micros();
	LPF_Id_obj.y_prev=0;
	LPF_Id_obj.Tf =  0.0005;
	
	
}

void set_up_PID(){
	
	//Sets up the Default Values for the PID Controller
	
	//PID Velocity
	PID_velocity_obj.timestamp_prev = micros();
	PID_velocity_obj.P = 0.0001;
	PID_velocity_obj._I = 2.0;
	PID_velocity_obj.D = 0.0;
	PID_velocity_obj.output_ramp = 1000;
	PID_velocity_obj.limit = PWM_obj.voltage_limit;
	PID_velocity_obj.integral_prev = 0.0;
	PID_velocity_obj.error_prev = 0.0 ;
	PID_velocity_obj.output_prev = 0.0;
	
	//PID Angle	
	PID_angle_obj.timestamp_prev = micros();
	PID_angle_obj.P = 2; //8
	PID_angle_obj._I = 4; //2
	PID_angle_obj.D = 0;
	PID_angle_obj.output_ramp = 1000;
	PID_angle_obj.limit = 4.0;
	PID_angle_obj.integral_prev = 0.0;
	PID_angle_obj.error_prev = 0.0 ;
	PID_angle_obj.output_prev = 0.0;
	
	//PID Current Q		
	PID_Iq_obj.timestamp_prev = micros();
	PID_Iq_obj.P = 3;
	PID_Iq_obj._I = 300;
	PID_Iq_obj.D = 0;
	PID_Iq_obj.output_ramp = 10;
	PID_Iq_obj.limit = PWM_obj.voltage_limit;
	PID_Iq_obj.integral_prev = 0.0;
	PID_Iq_obj.error_prev = 0.0 ;
	PID_Iq_obj.output_prev = 0.0;
	
	//PID Current D
	PID_Id_obj.timestamp_prev = micros();
	PID_Id_obj.P = 3;
	PID_Id_obj._I = 300;
	PID_Id_obj.D = 0;
	PID_Id_obj.output_ramp = 10;
	PID_Id_obj.limit = PWM_obj.voltage_limit;
	PID_Id_obj.integral_prev = 0.0;
	PID_Id_obj.error_prev = 0.0 ;
	PID_Id_obj.output_prev = 0.0;
	
}
	
float LPF(float value, LPF_struct *LPF_obj){
	unsigned long timestamp = micros();
	//unsigned long timestamp = 1;
	float dt = (timestamp - LPF_obj->timestamp_prev)*1e-6f;
	//For some overflow on the micros counter ??
	if (dt < 0.0f || dt > 0.5f){
		dt = 1e-3f;
	}
	float alpha = -LPF_obj->Tf/(LPF_obj->Tf + dt);
	float y = alpha*LPF_obj->y_prev + (1.0f - alpha)*value;
	LPF_obj->y_prev = y;
	LPF_obj->timestamp_prev = timestamp;
	return y;
}

float PID(float err, PID_struct *PID_obj){
	// calculate the time from the last call
	unsigned long timestamp_now = micros();
	float Ts = (timestamp_now - PID_obj->timestamp_prev) * 1e-6;
	// quick fix for strange cases (micros overflow)
	if(Ts <= 0 || Ts > 0.5) Ts = 1e-3;

	// u(s) = (P + I/s + Ds)e(s)
	// Discrete implementations
	// proportional part
	// u_p  = P *e(k)
	float proportional = PID_obj->P * err;
	// Tustin transform of the integral part
	// u_ik = u_ik_1  + I*Ts/2*(ek + ek_1)
	float integral = PID_obj->integral_prev +  PID_obj->_I*Ts*0.5*(err + PID_obj->error_prev);
	// antiwindup - limit the output
	integral = _constrain(integral, -PID_obj->limit,  PID_obj->limit);
	// Discrete derivation
	// u_dk = D(ek - ek_1)/Ts
	float derivative = PID_obj->D*(err - PID_obj->error_prev)/Ts;

	// sum all the components
	float output = proportional + integral + derivative;
	// antiwindup - limit the output variable
	output = _constrain(output, -PID_obj->limit, PID_obj->limit);

	// if output ramp defined
	if(PID_obj->output_ramp > 0){
		// limit the acceleration by ramping the output
		float output_rate = (output - PID_obj->output_prev)/Ts;
		if (output_rate > PID_obj->output_ramp){
			
			output = PID_obj->output_prev + PID_obj->output_ramp*Ts;
			
		}else if (output_rate < - PID_obj->output_ramp){
			
			output = PID_obj->output_prev - PID_obj->output_ramp*Ts;
		
		}
	}
	// saving for the next pass
	PID_obj->integral_prev = integral;
	PID_obj->output_prev = output;
	PID_obj->error_prev = err;
	PID_obj->timestamp_prev = timestamp_now;
	return output;
}

float _sqrtApprox(float number) {//low in fat
	long i;
	float y;
	// float x;
	// const float f = 1.5F; // better precision

	// x = number * 0.5F;
	y = number;
	i = * ( long * ) &y;
	i = 0x5f375a86 - ( i >> 1 );
	y = * ( float * ) &i;
	// y = y * ( f - ( x * y * y ) ); // better precision
	return number * y;
}
  
unsigned long millis( void )
{
	// todo: ensure no interrupts
	return _ulTickCount ;
}

void delay( unsigned long ms )
{
	if (ms == 0)
	{
		return;
	}

	uint32_t start = micros();

	while (ms > 0)
	{
		while (ms > 0 && (micros() - start) >= 1000)
		{
			ms--;
			start += 1000;
		}
	}
}

float getVelocity(float _current_angle){
	// calculate sample time
	unsigned long now_us = micros();
	
	
	
	float Ts = (now_us - velocity_calc_timestamp)*1e-6;
	// quick fix for strange cases (micros overflow)
	if(Ts <= 0 || Ts > 0.5) Ts = 1e-3;

	// current angle
	//float angle_c = getAngle();
	
	float angle_c = _current_angle;
	
	//float angle_c = get_shaft_angle_rad();
	
	// velocity calculation
	float vel = (angle_c - angle_prev)/Ts;



	// save variables for future pass
	angle_prev = angle_c;
	velocity_calc_timestamp = now_us;

	return vel;
}

void velocity_OL(float target_velocity){
	// get current timestamp
	unsigned long now_us = micros();
	// calculate the sample time from last call
	float Ts = (now_us - open_loop_timestamp) * 1e-6;
	// quick fix for strange cases (micros overflow + timestamp not defined)
	if(Ts <= 0 || Ts > 0.5){
		Ts = 1e-3;
		   
	}
	// calculate the necessary angle to achieve target velocity
	MOTOR_obj.shaft_angle = _normalizeAngle(MOTOR_obj.shaft_angle + target_velocity*Ts);
	
	
	//shaft_angle = _normalizeAngle(read_angle);
	// for display purposes
	//shaft_velocity = target_velocity;

	// use voltage limit or current limit
	float Uq = PWM_obj.voltage_limit;
	
	if(MOTOR_obj.phase_resistance > 0){
		Uq =  MOTOR_obj.current_limit*MOTOR_obj.phase_resistance;
	}
	
	
	modulate(Uq,  0, _electricalAngle(MOTOR_obj.shaft_angle,MOTOR_obj.pole_pairs));

	open_loop_timestamp = now_us;
}

float electricalAngle(float _angle){
	
	return _normalizeAngle(_angle * MOTOR_obj.pole_pairs - zero_electric_angle);
	
}

void velocity_CL(float target_velocity){
	
	if(MOTOR_obj.sensorless){

		//State Observer gets called here....

	}else{
		MOTOR_obj.shaft_angle = shaftAngle();
		MOTOR_obj.shaft_velocity = shaftVelocity(MOTOR_obj.shaft_angle);
		MOTOR_obj.electrical_angle = electricalAngle(MOTOR_obj.shaft_angle);
		
	}
	
	
	
	
	shaft_velocity_sp = target_velocity;
	//Calculate the Setpoint
	MOTOR_obj.current_sp = PID(shaft_velocity_sp - MOTOR_obj.shaft_velocity,&PID_velocity_obj);
	
	MOTOR_obj.U_q = MOTOR_obj.current_sp*MOTOR_obj.phase_resistance;
	MOTOR_obj.U_d = 0;
}

void Angle_CL(float target_angle){
	
	if(MOTOR_obj.sensorless){
	
		//State Observer gets called here....

	}else{

		MOTOR_obj.shaft_angle = shaftAngle();
		
		MOTOR_obj.shaft_velocity = shaftVelocity(MOTOR_obj.shaft_angle); // read value even if motor is disabled to keep the monitoring updated
		
		MOTOR_obj.electrical_angle = electricalAngle(MOTOR_obj.shaft_angle);
		
	}
	
	// angle set point
	shaft_angle_sp = target_angle;
	// calculate velocity set point
	shaft_velocity_sp = PID(shaft_angle_sp - MOTOR_obj.shaft_angle, &PID_angle_obj );

	MOTOR_obj.current_sp = shaft_velocity_sp;

	MOTOR_obj.U_q = MOTOR_obj.current_sp*MOTOR_obj.phase_resistance;
	
	MOTOR_obj.U_d = 0;

}

void Angle_OL(float target_angle){
	
	// get current timestamp
	unsigned long now_us = micros();
	// calculate the sample time from last call
	float Ts = (now_us - open_loop_timestamp) * 1e-6;
	// quick fix for strange cases (micros overflow + timestamp not defined)
	if(Ts <= 0 || Ts > 0.5) Ts = 1e-3;

	// calculate the necessary angle to move from current position towards target angle
	// with maximal velocity (velocity_limit)
	if(abs( target_angle - MOTOR_obj.shaft_angle ) > abs(velocity_limit*Ts)){
		MOTOR_obj.shaft_angle += _sign(target_angle - MOTOR_obj.shaft_angle) * abs( velocity_limit )*Ts;
		MOTOR_obj.shaft_velocity = velocity_limit;
		}else{
		MOTOR_obj.shaft_angle = target_angle;
		MOTOR_obj.shaft_velocity = 0;
	}
	
	// use voltage limit or current limit
	float Uq = PWM_obj.voltage_limit;
	
	if(MOTOR_obj.phase_resistance > 0){
		 Uq =  MOTOR_obj.current_limit*MOTOR_obj.phase_resistance;	 
	}


	
	
	//if(_isset(phase_resistance)) Uq =  current_limit*phase_resistance;
	// set the maximal allowed voltage (voltage_limit) with the necessary angle
	modulate(Uq,  0, _electricalAngle(MOTOR_obj.shaft_angle, MOTOR_obj.pole_pairs));

	// save timestamp for next call
	open_loop_timestamp = now_us;

}

void SinePWM(float Uq, float Ud, float angle_el){
	
	float _ca;
	float _sa;
	
	// Sinusoidal PWM modulation
	MOTOR_obj.U_d = Ud;
	MOTOR_obj.U_q = Uq;
	
	if(aprox_trig){
		angle_el = _normalizeAngle(angle_el);
		_ca = _cos(angle_el);
		_sa = _sin(angle_el);
	}else{
		_ca = cos(angle_el);
		_sa = sin(angle_el);
	}
	// Inverse park transform
	MOTOR_obj.U_alpha =  _ca * Ud - _sa * Uq;
	MOTOR_obj.U_beta =  _sa * Ud + _ca * Uq;
	
	float center = PWM_obj.voltage_limit/2;
	//float center = 6;
	


	// Inverse Clarke transform
	MOTOR_obj.U_phase_A = MOTOR_obj.U_alpha;
	MOTOR_obj.U_phase_B = -0.5 * MOTOR_obj.U_alpha  + _SQRT3_2 * MOTOR_obj.U_beta;
	MOTOR_obj.U_phase_C = -0.5 * MOTOR_obj.U_alpha - _SQRT3_2 * MOTOR_obj.U_beta;

	MOTOR_obj.U_alpha_prime = (2.f/3)*MOTOR_obj.U_phase_A- (1.f/3)*(MOTOR_obj.U_phase_B-MOTOR_obj.U_phase_C);;
	MOTOR_obj.U_beta_prime = _2_SQRT3*(MOTOR_obj.U_phase_B - MOTOR_obj.U_phase_C);



	MOTOR_obj.U_phase_A += center; 
	MOTOR_obj.U_phase_B += center;
	MOTOR_obj.U_phase_C += center;

	

	setPhaseVoltage(MOTOR_obj.U_phase_A,MOTOR_obj.U_phase_B,MOTOR_obj.U_phase_C);
	

	//Calculate Park Transform
	MOTOR_obj.U_d_prime = MOTOR_obj.U_alpha_prime * _ca + MOTOR_obj.U_beta_prime * _sa;
	MOTOR_obj.U_q_prime = MOTOR_obj.U_beta_prime * _ca - MOTOR_obj.U_alpha_prime * _sa;
		
		
	
}

void SpaceVectorPWM(float Uq, float Ud, float angle_el){
	
	MOTOR_obj.U_d = Ud;
	MOTOR_obj.U_q = Uq;
	
	int sector;
	
	// Nice video explaining the SpaceVectorModulation (SVPWM) algorithm
	// https://www.youtube.com/watch?v=QMSWUMEAejg

	// the algorithm goes
	// 1) Ualpha, Ubeta
	// 2) Uout = sqrt(Ualpha^2 + Ubeta^2)
	// 3) angle_el = atan2(Ubeta, Ualpha)
	//
	// equivalent to 2)  because the magnitude does not change is:
	//Uout = sqrt(Ud^2 + Uq^2);
	// equivalent to 3) is
	//angle_el = angle_el + atan2(Uq,Ud);

	float Uout;
	// a bit of optitmisation
	if(Ud){ // only if Ud and Uq set
		// _sqrt is an approx of sqrt (3-4% error)
		
		
		//Why is it divded by the voltage limit ?
		Uout = _sqrt(Ud*Ud + Uq*Uq) / PWM_obj.voltage_limit;
		// angle normalisation in between 0 and 2pi
		// only necessary if using _sin and _cos - approximation functions
		angle_el = _normalizeAngle(angle_el + atan2(Uq, Ud));
	}else{// only Uq available - no need for atan2 and sqrt
		Uout = Uq / PWM_obj.voltage_limit;
		// angle normalisation in between 0 and 2pi
		// only necessary if using _sin and _cos - approximation functions
		angle_el = _normalizeAngle(angle_el + _PI_2);
	}
	// find the sector we are in currently
	sector = floor(angle_el / _PI_3) + 1;
	// calculate the duty cycles
	float T1 = _SQRT3*_sin(sector*_PI_3 - angle_el) * Uout;
	float T2 = _SQRT3*_sin(angle_el - (sector-1.0)*_PI_3) * Uout;
	// two versions possible
	float T0 = 0; // pulled to 0 - better for low power supply voltage
	
	if (1) {
		T0 = 1 - T1 - T2; //modulation_centered around driver->voltage_limit/2
	}

	// calculate the duty cycles(times)
	float Ta,Tb,Tc;
	switch(sector){
		case 1:
		Ta = T1 + T2 + T0/2;
		Tb = T2 + T0/2;
		Tc = T0/2;
		break;
		case 2:
		Ta = T1 +  T0/2;
		Tb = T1 + T2 + T0/2;
		Tc = T0/2;
		break;
		case 3:
		Ta = T0/2;
		Tb = T1 + T2 + T0/2;
		Tc = T2 + T0/2;
		break;
		case 4:
		Ta = T0/2;
		Tb = T1+ T0/2;
		Tc = T1 + T2 + T0/2;
		break;
		case 5:
		Ta = T2 + T0/2;
		Tb = T0/2;
		Tc = T1 + T2 + T0/2;
		break;
		case 6:
		Ta = T1 + T2 + T0/2;
		Tb = T0/2;
		Tc = T1 + T0/2;
		break;
		default:
		// possible error state
		Ta = 0;
		Tb = 0;
		Tc = 0;
		
	}

	// calculate the phase voltages and center
	MOTOR_obj.U_phase_A = Ta*PWM_obj.voltage_limit;
	MOTOR_obj.U_phase_B = Tb*PWM_obj.voltage_limit;
	MOTOR_obj.U_phase_C = Tc*PWM_obj.voltage_limit;
	
	//Push Calculated Voltages outputs
	setPhaseVoltage(MOTOR_obj.U_phase_A,MOTOR_obj.U_phase_B,MOTOR_obj.U_phase_C);
}

void delayMicroseconds(unsigned int us)
{
	
	SET(LED1)
	
	if (us == 0)
	return;
	
	
	
	uint32_t start, elapsed;
	uint32_t count;

	
	count = us * (F_CPU / 1000000) - 20;  // convert us to cycles.
	start = DWT->CYCCNT;  //CYCCNT is 32bits, takes 37s or so to wrap.
	SET(LED1)
	while (1) {
		elapsed = DWT->CYCCNT - start;
		if (elapsed >= count)
		return;
	}
	RESET(LED1)
	

	
}

unsigned long micros( void )
{
	
	// Interrupt-compatible version of micros
	// Theory: repeatedly take readings of SysTick counter, millis counter and SysTick interrupt pending flag.
	// When it appears that millis counter and pending is stable and SysTick hasn't rolled over, use these
	// values to calculate micros. If there is a pending SysTick, add one to the millis counter in the calculation.

	uint32_t ticks, ticks2;
	uint32_t pend, pend2;
	uint32_t count, count2;

	ticks2  = SysTick->VAL;
	pend2   = !!(SCB->ICSR & SCB_ICSR_PENDSTSET_Msk)  ;
	count2  = _ulTickCount ;

	do
	{
		ticks=ticks2;
		pend=pend2;
		count=count2;
		ticks2  = SysTick->VAL;
		pend2   = !!(SCB->ICSR & SCB_ICSR_PENDSTSET_Msk)  ;
		count2  = _ulTickCount ;
	} while ((pend != pend2) || (count != count2) || (ticks < ticks2));

	return ((count+pend) * 1000) + (((SysTick->LOAD  - ticks)*(1048576/(F_CPU/1000000)))>>20) ;
	// this is an optimization to turn a runtime division into two compile-time divisions and
	// a runtime multiplication and shift, saving a few cycles
}

void setPhaseVoltage(float U_A, float U_B, float U_C){
	
	//Setting the Voltage Limit 
	U_A = _constrain(U_A, 0.0, PWM_obj.voltage_limit);
	U_B = _constrain(U_B, 0.0, PWM_obj.voltage_limit);
	U_C = _constrain(U_C, 0.0, PWM_obj.voltage_limit);
	
	
	//Calculating the Duty Cycles
	float dc_a = _constrain(U_A / PWM_obj.voltage_power_supply, 0.0 , 1.0 );
	float dc_b = _constrain(U_B / PWM_obj.voltage_power_supply, 0.0 , 1.0 );
	float dc_c = _constrain(U_C / PWM_obj.voltage_power_supply, 0.0 , 1.0 );
	
	//Pushing the Duty cycles to the hardware
	//while ( (TCC0->SYNCBUSY.vec.CC & (0x1<<chan)) > 0 );
	while ( TCC0->SYNCBUSY.vec.CC );
	uint32_t adc = (uint32_t)((PWM_obj.PWM_resolution-1) * dc_c);
	
	TCC0->CCBUF[0].reg = adc;
	//while(TCC0->SYNCBUSY.bit.CC0);

	while ( TCC0->SYNCBUSY.vec.CC );
	TCC0->CCBUF[1].reg = (uint32_t)((PWM_obj.PWM_resolution-1) * dc_b);
	//while(TCC0->SYNCBUSY.bit.CC1);

	while ( TCC0->SYNCBUSY.vec.CC );
	TCC0->CCBUF[2].reg = (uint32_t)((PWM_obj.PWM_resolution-1) * dc_a);
	//while(TCC0->SYNCBUSY.bit.CC2);

}

float _cos(float a){
	float a_sin = a + _PI_2;
	a_sin = a_sin > _2PI ? a_sin - _2PI : a_sin;
	return _sin(a_sin);
}

float _sin(float a){
	if(a < _PI_2){
		//return sine_array[(int)(199.0*( a / (_PI/2.0)))];
		//return sine_array[(int)(126.6873* a)];           // float array optimized
		return 0.0001*sine_array[_round(126.6873* a)];      // int array optimized
		}else if(a < _PI){
		// return sine_array[(int)(199.0*(1.0 - (a-_PI/2.0) / (_PI/2.0)))];
		//return sine_array[398 - (int)(126.6873*a)];          // float array optimized
		return 0.0001*sine_array[398 - _round(126.6873*a)];     // int array optimized
		}else if(a < _3PI_2){
		// return -sine_array[(int)(199.0*((a - _PI) / (_PI/2.0)))];
		//return -sine_array[-398 + (int)(126.6873*a)];           // float array optimized
		return -0.0001*sine_array[-398 + _round(126.6873*a)];      // int array optimized
		} else {
		// return -sine_array[(int)(199.0*(1.0 - (a - 3*_PI/2) / (_PI/2.0)))];
		//return -sine_array[796 - (int)(126.6873*a)];           // float array optimized
		return -0.0001*sine_array[796 - _round(126.6873*a)];      // int array optimized
	}
}

float _normalizeAngle(float angle){
	float a = fmod(angle, _2PI);
	return a >= 0 ? a : (a + _2PI);
}

// Electrical angle calculation
float _electricalAngle(float shaft_angle, int pole_pairs) {
	return (shaft_angle * pole_pairs);
}

void TC0_Handler(){
	//Reset the overflow flag should be at the bottom of the code.
	TC0->COUNT16.INTFLAG.bit.OVF=1;

}


volatile int tcc_skip_count = 0;
void TCC0_0_Handler(){
	
	//This interupt should run everytime TCC0 counter is at 0 
	//This means all Low Side FETS are active.

	//if(CURRENT_obj.read_currents){	

	
	CURRENT_obj.I_raw_phase_A = ADC_Read(PHASE_A);
	CURRENT_obj.I_raw_phase_B = ADC_Read(PHASE_B);
	//CURRENT_obj.I_raw_phase_C = ADC_Read(PHASE_A);
	//Currents Have been read, resetting the flag
	CURRENT_obj.read_currents=FALSE;
		
	//}
	
	
	//Currents have been read, we now convert them from RAW form to Voltage form
	CURRENT_obj.I_raw_phase_A_voltage = CURRENT_obj.I_raw_phase_A*CURRENT_obj.conversion_scalar;
	CURRENT_obj.I_raw_phase_B_voltage = CURRENT_obj.I_raw_phase_B*CURRENT_obj.conversion_scalar;
	//CURRENT_obj.I_raw_phase_C_voltage = CURRENT_obj.I_raw_phase_C*CURRENT_obj.conversion_scalar;
	
	//CURRENT_obj.I_raw_phase_C_voltage = CURRENT_obj.I_raw_phase_A_voltage + CURRENT_obj.I_raw_phase_B_voltage;

	//We now subtract the offset of the current sensing and apply the Voltage to Current conversion
	MOTOR_obj.I_phase_A = (CURRENT_obj.I_raw_phase_A_voltage - CURRENT_obj.I_phase_A_offset)*CURRENT_obj.I_gain_phase_A;// amps
	MOTOR_obj.I_phase_B = (CURRENT_obj.I_raw_phase_B_voltage - CURRENT_obj.I_phase_B_offset)*CURRENT_obj.I_gain_phase_B;// amps
	//MOTOR_obj.I_phase_C = (CURRENT_obj.I_raw_phase_C_voltage - CURRENT_obj.I_phase_C_offset)*CURRENT_obj.I_gain_phase_C;// amps
	
	MOTOR_obj.I_phase_C = MOTOR_obj.I_phase_A +  MOTOR_obj.I_phase_B;

	/*	
	
	buffer[buffer_pos++]=MOTOR_obj.I_phase_C;
	if (buffer_pos==sizeof(buffer)/sizeof(buffer[0])){
		buffer_pos=0;
		usb_send_bulk_data((uint8_t*)buffer,sizeof(buffer),NULL);
	}
	
	*/
	
	//Reset the overflow flag should be at the bottom of the code.
	TCC0->INTFLAG.bit.OVF=1;
}

void TCC0_setup(){
	
	//Enable the peripheral channel for TCC0
	//GCLK->PCHCTRL[GCLK_TCC0].reg= GCLK_PCHCTRL_CHEN;
	
	//TCC0 will get its clock from generator 4 which has 48MHz
	GCLK->PCHCTRL[GCLK_TCC0].reg = GCLK_PCHCTRL_CHEN | GCLK_PCHCTRL_GEN(GENERIC_CLOCK_GENERATOR_48M);
	
	while(GCLK->SYNCBUSY.reg & GCLK_SYNCBUSY_GENCTRL_GCLK1);
	
	//enable that master clock to TCC0 ... page 188
	MCLK->APBBMASK.reg |= MCLK_APBBMASK_TCC0;
	
	//Resetting the counter 
	TCC0->CTRLA.reg = TCC_CTRLA_SWRST;
	
	while(TCC0->SYNCBUSY.reg & TCC_SYNCBUSY_SWRST);
	
	//Setting Prescaler to 1
	TCC0->CTRLA.reg = TCC_CTRLA_PRESCALER_DIV1;
	
	//Wave form generation set to normal PWM
	//TCC0->WAVE.reg |= TCC_WAVE_WAVEGEN_NPWM;

	
	//Wave form generation set to normal PWM
	//TCC0->WAVE.reg = TCC_WAVE_WAVEGEN_DSBOTTOM;
	
	//TCC0->WAVE.reg |= TCC_WAVE_POL(0xF)|TCC_WAVE_WAVEGEN_DSTOP;

	//TCC0->WAVE.reg |= TCC_WAVE_POL(0xF)|TCC_WAVE_WAVEGEN_DSBOTTOM;

	//Interupt Will on Trigger when PER is at ZERO
	TCC0->WAVE.reg |= TCC_WAVE_WAVEGEN_DSBOTTOM;
	
	//TCC0->WAVE.bit.WAVEGEN = TCC_WAVE_WAVEGEN_DSCRITICAL;
	
	

	
	while ( TCC0->SYNCBUSY.bit.WAVE == 1 );
	
	//Setting the TOP value of the counter
	TCC0->PER.reg = PWM_obj.PWM_resolution;
	
	while ( TCC0->SYNCBUSY.bit.PER == 1 );
	
	//TCC0->WAVE.reg |= TCC_WAVE_POL(0xF)
	
	
	TCC0->WEXCTRL.reg |= TCC_WEXCTRL_DTHS(50);
		
	TCC0->WEXCTRL.reg |= TCC_WEXCTRL_DTLS(50);
	
	TCC0->WEXCTRL.reg |= TCC_WEXCTRL_DTIEN0;
	TCC0->WEXCTRL.reg |= TCC_WEXCTRL_DTIEN1;
	TCC0->WEXCTRL.reg |= TCC_WEXCTRL_DTIEN2;
		
	//Set the output MATRIX to 0
	TCC0->WEXCTRL.reg |= TCC_WEXCTRL_OTMX(0x0);

	//Not sure about the Sync for this register.
	TCC0->WAVE.reg |= TCC_WAVE_SWAP0;
	while(TCC0->SYNCBUSY.bit.WAVE);
	TCC0->WAVE.reg |= TCC_WAVE_SWAP1;
	while(TCC0->SYNCBUSY.bit.WAVE);
	TCC0->WAVE.reg |= TCC_WAVE_SWAP2;	
	while(TCC0->SYNCBUSY.bit.WAVE);
	
	
	/*
	TCC0->WAVE.reg |= TCC_WAVE_SWAP3;
	*/

	TCC0->CC[0].reg = 0;
	while(TCC0->SYNCBUSY.bit.CC0);
	TCC0->CC[1].reg = 0;
	while(TCC0->SYNCBUSY.bit.CC1);
	TCC0->CC[2].reg = 0;
	while(TCC0->SYNCBUSY.bit.CC2);
	
	//When the overflow is detected the interupt flag is raised...
	TCC0->INTENSET.bit.OVF=1;
	
	

	TCC0->CTRLA.reg |= TCC_CTRLA_ENABLE;
	
	while(TCC0->SYNCBUSY.reg & TCC_SYNCBUSY_ENABLE);
	
	NVIC_EnableIRQ(TCC0_0_IRQn);

	
}

void set_TC0(){
	//Page 169
	//We map the global clock to the TC0 block. GEN=4 => clock generator 4 to TC0 12MHz
	GCLK->PCHCTRL[GCLK_TC0].bit.GEN=4;
	//CHEN:?Channel Enable ... enable the channel
	GCLK->PCHCTRL[GCLK_TC0].bit.CHEN=1;
	//Wait until this channel is enabled
	while (!GCLK->PCHCTRL[GCLK_TC0].bit.CHEN);
	
	//The APBA clock for the TC0 is enabled.
	MCLK->APBAMASK.bit.TC0_=1;
	
	
	//Sets the value of the compare register... IE when the overflow will occur
	//The ISR should be called every microsecond....
	TC0->COUNT16.CC[0].bit.CC=300;
	//TC0->COUNT16.CC[0].bit.CC=11718;
	//TC0->COUNT16.CC[0].bit.CC=100;
	//We set the prescaler to 1024
	//TC0->COUNT16.CTRLA.bit.PRESCALER=0x0; // DIV1
	//TC0->COUNT16.CTRLA.bit.PRESCALER=0x4; // DIV16
	TC0->COUNT16.CTRLA.bit.PRESCALER=0x6; // DIV256
	//TC0->COUNT16.CTRLA.bit.PRESCALER=0x7; // DIV1024
		
	//WAVEGEN=1 => Match Frequency Operation.
	TC0->COUNT16.WAVE.bit.WAVEGEN=1; // top is in CC0
	
	//When the overflow is detected the interupt flag is raised...
	TC0->COUNT16.INTENSET.bit.OVF=1;
		
	//Enables the counter 
	TC0->COUNT16.CTRLA.bit.ENABLE=1; // enables TC0
		
	//Wait for the block to syncronize
	while(TC0->COUNT16.SYNCBUSY.bit.ENABLE);
		
	//After an overflow has occured. Reset the timer ??
	TC0->COUNT16.CTRLBSET.bit.CMD=1; // Force start
		
	//Somthing for the interupt service routine...
	NVIC_EnableIRQ(TC0_IRQn);
		
}

void PINs_setup(){
	
	
	//Page 819 to set the pin as input and pull-down enabled...
	
	//Buttons are Inputs
	DIR_IN(BTN1)
	DIR_IN(BTN2)
	
	// Actually Sets the Pull-down Resistor since the DIRCLR reg is set from DIR_IN macro.
	PULLUP_ENABLE(BTN1) 
	PULLUP_ENABLE(BTN2)
	
	//The current sense pins will be inputs.
	DIR_IN(I_PHASE_1)
	DIR_IN(I_PHASE_2)
	DIR_IN(I_PHASE_3)
	
	//SPI stuff
	/*
	DIR_IN(MISO)
	DIR_OUT(MOSI)
	DIR_OUT(CLK)
	
*/
	DIR_OUT(CS)
	
	
	//SPI EXTRA
	
	DIR_OUT(EX_1)
	DIR_OUT(EX_2)
	DIR_OUT(EX_3)
	DIR_OUT(EX_4)
	
	
	//LEDs are outputs
	DIR_OUT(LED1)
	DIR_OUT(LED2)
	DIR_OUT(LED4)
	DIR_OUT(LED5)
	
	//Phase controls are outputs
	DIR_OUT(P1_HS_LOGIC)
	DIR_OUT(P2_HS_LOGIC)
	DIR_OUT(P3_HS_LOGIC)
	DIR_OUT(P1_LS_LOGIC)
	DIR_OUT(P2_LS_LOGIC)
	DIR_OUT(P3_LS_LOGIC)
	
	//Enabling alternative functions
	PORT_ALT_FCN_ENABLE(I_PHASE_1)
	PORT_ALT_FCN_ENABLE(I_PHASE_2)
	PORT_ALT_FCN_ENABLE(I_PHASE_3)
	
	PORT_ALT_FCN_ENABLE(P1_HS_LOGIC)
	PORT_ALT_FCN_ENABLE(P2_HS_LOGIC)
	PORT_ALT_FCN_ENABLE(P3_HS_LOGIC)

	PORT_ALT_FCN_ENABLE(P1_LS_LOGIC)
	PORT_ALT_FCN_ENABLE(P2_LS_LOGIC)
	PORT_ALT_FCN_ENABLE(P3_LS_LOGIC)

	PORT_ALT_FCN_ENABLE(USBP)
	PORT_ALT_FCN_ENABLE(USBM)
	
	PORT_ALT_FCN_ENABLE(MISO)
	PORT_ALT_FCN_ENABLE(CLK)
	PORT_ALT_FCN_ENABLE(MOSI)
	//PORT_ALT_FCN_ENABLE(CS)
	
	//USB goes to the USB block H
	PORT_MUX(USBP,PORT_MUX_H)
	PORT_MUX(USBM,PORT_MUX_H)
	
	//Current reading for to Block B - ADC0
	PORT_MUX(I_PHASE_1,PORT_MUX_B);
	PORT_MUX(I_PHASE_2,PORT_MUX_B);
	PORT_MUX(I_PHASE_3,PORT_MUX_B);
	
	//MOSFET control to block G - TCC0
	PORT_MUX(P1_HS_LOGIC,PORT_MUX_G)
	PORT_MUX(P2_HS_LOGIC,PORT_MUX_G)
	PORT_MUX(P3_HS_LOGIC,PORT_MUX_G)
	PORT_MUX(P1_LS_LOGIC,PORT_MUX_G)
	PORT_MUX(P2_LS_LOGIC,PORT_MUX_G)
	PORT_MUX(P3_LS_LOGIC,PORT_MUX_G)
	
	//SPI COM to block D - SERCOM 5
	
	PORT_MUX(MISO,PORT_MUX_D)
	PORT_MUX(CLK,PORT_MUX_D)
	PORT_MUX(MOSI,PORT_MUX_D)
	//PORT_MUX(CS,PORT_MUX_D)

}

uint16_t AS5048A_send_command(int cmd){
	//We read the raw angle data from the AS5048A
	//Stores the incoming data
	uint16_t readdata =0;
	//Command that will be sent to the AS5048A, the 15th bit = 1 -> read command
	uint16_t command = 0b0100000000000000; // PAR=0 R/W=R
	//We do a or operation with the given command
	command |= cmd;
	//Add a parity bit on the the MSB
	command |= ((uint16_t)CalcEvenParity(command)<<15);
	//Sends the first command to the AS5048A
	
	RESET(CS);
	readdata = SPI_transfer_16_bits(command);
	SET(CS);
	delay(1);
	//We ignore the first data received since the chip needs to process the command we have sent
	//We wait
	
	//Send the same command again
	
	RESET(CS);
	readdata = SPI_transfer_16_bits(command);
	SET(CS);
	//return the data received and also remove the parity bit and read bit
	
	//We check if the error bit is high...
	if (bitRead(readdata,14)) {
		SET(LED5)
		error_flag = 1;
	}else {
		RESET(LED5)
		error_flag = 0;
	}
	//We remove the parity and error bit from the data...
	return readdata & ~0xC000;
}

char CalcEvenParity(uint16_t value){
	char cnt = 0;
	char i;

	for (i = 0; i < 16; i++)
	{
		if (value & 0x1){
			cnt++;
		}
		value >>= 1;
	}
	return cnt & 0x1;
}

uint8_t  SPI_transfer_8_bits(uint8_t data){
	
	while(SERCOM5->SPI.INTFLAG.bit.DRE == 0);
	//We push 8 bits to the AS5048A
	//SERCOM5->SPI.DATA.bit.DATA = data; // Writing data into Data register
	
	SERCOM5->SPI.DATA.reg = data;	
	
	//Wait for data to come back
	while(SERCOM5->SPI.INTFLAG.bit.RXC == 0); // Waiting Complete Reception
	//Return the data that was received
	
	//return SERCOM5->SPI.DATA.bit.DATA;  // Reading data
	
	return SERCOM5->SPI.DATA.reg;
}

uint16_t SPI_transfer_16_bits(uint16_t data){
	//The AS5048A take 16 bit command, we can only send 8bits at a time
	//We split the 16bit value into 2x8bit variables in the structure below
	union {
		uint16_t val;
		struct {
			uint8_t lsb;
			uint8_t msb;
		};
	} t;
	//We put the data into the structure
	t.val = data;
	//We send the MSB (first part)
	t.msb = SPI_transfer_8_bits(t.msb);
	//We second the LSB (second part)
	t.lsb = SPI_transfer_8_bits(t.lsb);
	//We return what the AS5048A sent back
	return t.val;
}

void set_up_SPI_SERCOM_5(uint32_t baudrate){
	
	//Main Clock to SERCOM5
	MCLK->APBDMASK.reg |= MCLK_APBDMASK_SERCOM5;
	//Wait for SPI in case it is doing something
	while(SERCOM5->SPI.SYNCBUSY.bit.ENABLE);
	//Disable SPI
	SERCOM5->SPI.CTRLA.bit.ENABLE = 0;
	//Software Reset SPI
	SERCOM5->SPI.CTRLA.bit.SWRST = 1;
	//Wait both bits Software Reset from CTRLA and SYNCBUSY are equal to 0
	while(SERCOM5->SPI.CTRLA.bit.SWRST || SERCOM5->SPI.SYNCBUSY.bit.SWRST);
	// Disable core timer
	GCLK->PCHCTRL[SERCOM5_GCLK_ID_CORE].bit.CHEN = 0;
	// Wait for disable
	while(GCLK->PCHCTRL[SERCOM5_GCLK_ID_CORE].bit.CHEN);
	// Disable slow timer
	GCLK->PCHCTRL[SERCOM5_GCLK_ID_SLOW].bit.CHEN = 0;
	// Wait for disable
	while(GCLK->PCHCTRL[SERCOM5_GCLK_ID_SLOW].bit.CHEN);
	//SERCOM5_GCLK_ID_CORE gets 48 MHz from Gen CLK 1
	GCLK->PCHCTRL[SERCOM5_GCLK_ID_CORE].reg = GCLK_PCHCTRL_GEN_GCLK1_Val |
	(1 << GCLK_PCHCTRL_CHEN_Pos);
	//Sync Clock
	while(!GCLK->PCHCTRL[SERCOM5_GCLK_ID_CORE].bit.CHEN); // Wait for core clock enable
	//SERCOM5_GCLK_ID_SLOW gets 48 MHz from Gen CLK 1
	GCLK->PCHCTRL[SERCOM5_GCLK_ID_SLOW].reg = GCLK_PCHCTRL_GEN_GCLK1_Val |
	(1 << GCLK_PCHCTRL_CHEN_Pos);
	
	while(!GCLK->PCHCTRL[SERCOM5_GCLK_ID_SLOW].bit.CHEN); // Wait for slow clock enable
	//Sync Clock
	while(GCLK->SYNCBUSY.reg & GCLK_SYNCBUSY_GENCTRL_GCLK1);
	//Frame Format = 0 -> SPI Frame
	SERCOM5->SPI.CTRLA.bit.FORM = 0;
	SERCOM5->SPI.CTRLA.reg = SERCOM_SPI_CTRLA_MODE(0x3)  |  // Master mode
	SERCOM_SPI_CTRLA_DOPO(0x0) | // MOSI on PAD[0] , SCK on PAD[1], SS on PAD[2]
	SERCOM_SPI_CTRLA_DIPO(0x3) | // MISO on PAD[3]
	0 << SERCOM_SPI_CTRLA_DORD_Pos; // MSB is transfered first
	SERCOM5->SPI.CTRLB.reg = SERCOM_SPI_CTRLB_CHSIZE(0x0) | //8 bits Char Size
	SERCOM_SPI_CTRLB_RXEN; //Active the SPI receiver.
	//Sercom will handle the toggling of the Chip Select Pin
	//SERCOM5->SPI.CTRLB.bit.MSSEN =1;
	//Syncing CTRLB
	while( SERCOM5->SPI.SYNCBUSY.bit.CTRLB == 1 );
	// CPOL = 0 -> Clock is low on Idle. The leading edge of a clock cycle is a rising edge, while the trailing
	// edge is a falling edge.
	// CPHA = 1 -> The data is sampled on a trailing SCK edge and changed on a leading SCK edge.
	SERCOM5->SPI.CTRLA.reg |= ( 1 << SERCOM_SPI_CTRLA_CPHA_Pos ) |
	(0  << SERCOM_SPI_CTRLA_CPOL_Pos );
	//Calculating the baud rate... equation from data sheet
	uint16_t b = (SERCOM_SPI_FREQ_REF / (2 * baudrate))-1;
	//Putting calculated baud rate into the register
	SERCOM5->SPI.BAUD.reg = b;
	//Enable SPI
	SERCOM5->SPI.CTRLA.bit.ENABLE = 1;
	//Wait for Sync
	while(SERCOM5->SPI.SYNCBUSY.bit.ENABLE);
}

void set_up_clocks(){
	
	
	/*
	 * Function:  set_up_clocks
	 * --------------------
	*This sets up the clocks of the chip
	*Gen Clock 3 gets 32khz from Internal Oscillator XOSC32K
	*DFLL48MHz Runs in Open loop mode
	*Genclock 5 runs at 1Mhz
	*PLL0 runs at 120MHz
	*PLL1 runs at 100MHz
	*Gen Clock 1 runs at 48MHz
	*Configure Gen Clock 2 to 100MHz
	*Configure Gen Clock 4 to 12MHz
	*Gen Clock 0 to 120MHz -> CPU runs at 120MHz
	*/
	//For now we will be using the internal 32kHz oscillator...
	//Put Internal oscillator to Gen Clock 3
	NVMCTRL->CTRLA.reg |= NVMCTRL_CTRLA_RWS(0);
	
	GCLK->GENCTRL[GENERIC_CLOCK_GENERATOR_XOSC32K].reg = GCLK_GENCTRL_SRC(GCLK_GENCTRL_SRC_OSCULP32K) | //generic clock gen 3
	GCLK_GENCTRL_GENEN;
	while ( GCLK->SYNCBUSY.reg & GCLK_SYNCBUSY_GENCTRL3 );
	
	//Put 32kHz to the main Clock...might not be needed ?
	GCLK->GENCTRL[GENERIC_CLOCK_GENERATOR_MAIN].reg = GCLK_GENCTRL_SRC(GCLK_GENCTRL_SRC_OSCULP32K) | GCLK_GENCTRL_GENEN;
	while ( GCLK->SYNCBUSY.reg & GCLK_SYNCBUSY_GENCTRL0 );
	
	//Configure DFLL48MHz in Open Loop Mode... 
	OSCCTRL->DFLLCTRLA.reg = 0; // Clear the Reg...
	OSCCTRL->DFLLMUL.reg = OSCCTRL_DFLLMUL_CSTEP( 0x1 ) | //Course Step = 1
	OSCCTRL_DFLLMUL_FSTEP( 0x1 ) | // Fine Step = 1
	OSCCTRL_DFLLMUL_MUL( 0 ); // Multiplication Factor = 0
	  
	while ( OSCCTRL->DFLLSYNC.reg & OSCCTRL_DFLLSYNC_DFLLMUL );
	  
	OSCCTRL->DFLLCTRLB.reg = 0; // Clear the Reg...
	while ( OSCCTRL->DFLLSYNC.reg & OSCCTRL_DFLLSYNC_DFLLCTRLB );
	  
	OSCCTRL->DFLLCTRLA.reg |= OSCCTRL_DFLLCTRLA_ENABLE; // Enable DFLL
	while ( OSCCTRL->DFLLSYNC.reg & OSCCTRL_DFLLSYNC_ENABLE );
	
	  
	OSCCTRL->DFLLVAL.reg = OSCCTRL->DFLLVAL.reg; // Copy Paste ?
	while( OSCCTRL->DFLLSYNC.bit.DFLLVAL );
	  
	OSCCTRL->DFLLCTRLB.reg = OSCCTRL_DFLLCTRLB_WAITLOCK | // Enable Fine Lock ?
	OSCCTRL_DFLLCTRLB_CCDIS | // Disable Chill Cycle ?
	OSCCTRL_DFLLCTRLB_USBCRM ; //Enable USB clock recovery mode
	while ( !OSCCTRL->STATUS.bit.DFLLRDY );
	//DFLL48MHz Enabled
	
	//Put 1MHz to Genclock 5
	GCLK->GENCTRL[GENERIC_CLOCK_GENERATOR_1M].reg = GCLK_GENCTRL_SRC(GCLK_GENCTRL_SRC_DFLL_Val) | //Source from DFLL48MHz
	GCLK_GENCTRL_GENEN | //Enable
	GCLK_GENCTRL_DIV(48u); // Divide by 48 -> 48Mhz/48 = 1MHz
	while ( GCLK->SYNCBUSY.bit.GENCTRL5 );	  
	
	
	//Configure PLL0 to 120MHz
	GCLK->PCHCTRL[OSCCTRL_GCLK_ID_FDPLL0].reg = (1 << GCLK_PCHCTRL_CHEN_Pos) | //Enable output
	GCLK_PCHCTRL_GEN(GCLK_PCHCTRL_GEN_GCLK5_Val); // Reference from Genclock 5 running at 1MHz
	  
	// This rounds to nearest full-MHz increment; not currently using frac
	OSCCTRL->Dpll[0].DPLLRATIO.reg = OSCCTRL_DPLLRATIO_LDRFRAC(0x00) | // Loop Divider Ratio Fraction = 0
	OSCCTRL_DPLLRATIO_LDR((F_CPU - 500000) / 1000000); // Loop Divider Ratio Integer = 119
	while(OSCCTRL->Dpll[0].DPLLSYNCBUSY.bit.DPLLRATIO);
	
	OSCCTRL->Dpll[0].DPLLCTRLB.reg = OSCCTRL_DPLLCTRLB_REFCLK_GCLK | // Reference from Generic Clock
	OSCCTRL_DPLLCTRLB_LBYPASS; // Enable Lock Bypass
	OSCCTRL->Dpll[0].DPLLCTRLA.reg = OSCCTRL_DPLLCTRLA_ENABLE; // Enable DPLL0
	while( OSCCTRL->Dpll[0].DPLLSTATUS.bit.CLKRDY == 0 || OSCCTRL->Dpll[0].DPLLSTATUS.bit.LOCK == 0 );
	//PLL0 set to 120MHz and enabled
	
	//PLL1 is 100MHz
	GCLK->PCHCTRL[OSCCTRL_GCLK_ID_FDPLL1].reg = (1 << GCLK_PCHCTRL_CHEN_Pos) | //Enable output
	GCLK_PCHCTRL_GEN(GCLK_PCHCTRL_GEN_GCLK5_Val); // Reference from Genclock 5 running at 1MHz
	
	OSCCTRL->Dpll[1].DPLLRATIO.reg = OSCCTRL_DPLLRATIO_LDRFRAC(0x00) | // Loop Divider Ratio Fraction = 0
	OSCCTRL_DPLLRATIO_LDR(99);  // Loop Divider Ratio Integer = 99
	while(OSCCTRL->Dpll[1].DPLLSYNCBUSY.bit.DPLLRATIO);
	
	OSCCTRL->Dpll[1].DPLLCTRLB.reg = OSCCTRL_DPLLCTRLB_REFCLK_GCLK | // Reference from Generic Clock
	OSCCTRL_DPLLCTRLB_LBYPASS; // Enable Lock Bypass
	OSCCTRL->Dpll[1].DPLLCTRLA.reg = OSCCTRL_DPLLCTRLA_ENABLE;
	while( OSCCTRL->Dpll[1].DPLLSTATUS.bit.CLKRDY == 0 || OSCCTRL->Dpll[1].DPLLSTATUS.bit.LOCK == 0 );
	//PLL1 set to 100MHz and enabled
	
	//Configure Gen Clock 1 to 48MHz
	GCLK->GENCTRL[GENERIC_CLOCK_GENERATOR_48M].reg = GCLK_GENCTRL_SRC(GCLK_GENCTRL_SRC_DFLL_Val) | // Source From DFLL48MHz
	GCLK_GENCTRL_IDC | // Enable balanced duty cycle
	GCLK_GENCTRL_GENEN; // Enable output
	while ( GCLK->SYNCBUSY.reg & GENERIC_CLOCK_GENERATOR_48M_SYNC);
	//Gen Clock 1 set to 48MHz and enabled
	
	
	//Configure Gen Clock 2 to 100MHz
	GCLK->GENCTRL[GENERIC_CLOCK_GENERATOR_100M].reg = GCLK_GENCTRL_SRC(GCLK_GENCTRL_SRC_DPLL1_Val)| // Source From PLL1
	GCLK_GENCTRL_IDC | // Enable balanced duty cycle
	GCLK_GENCTRL_GENEN; // Enable output
	while ( GCLK->SYNCBUSY.reg & GENERIC_CLOCK_GENERATOR_100M_SYNC);
	//Gen Clock 2 set to 100MHz and enabled
	
	
	//Configure Gen Clock 4 to 12MHz
	GCLK->GENCTRL[GENERIC_CLOCK_GENERATOR_12M].reg = GCLK_GENCTRL_SRC(GCLK_GENCTRL_SRC_DFLL_Val)| // Source From DFLL48MHz
	GCLK_GENCTRL_IDC | // Enable balanced duty cycle
	GCLK_GENCTRL_DIV(4)| //Divide input by 4 to get 12MHz
	GCLK_GENCTRL_GENEN;  //Enable output
	while ( GCLK->SYNCBUSY.reg & GENERIC_CLOCK_GENERATOR_12M_SYNC);
	//Gen Clock 4 set to 12MHz and enabled
	
	
	//Gen Clock 1 to 120MHz
	GCLK->GENCTRL[GENERIC_CLOCK_GENERATOR_MAIN].reg = GCLK_GENCTRL_SRC(MAIN_CLOCK_SOURCE) | // Gen Clock 0 gets source from DPLL0
    GCLK_GENCTRL_IDC | // Enable balanced duty cycle
    GCLK_GENCTRL_GENEN; //Enable output
	while ( GCLK->SYNCBUSY.reg & GCLK_SYNCBUSY_GENCTRL0 );
	MCLK->CPUDIV.reg = MCLK_CPUDIV_DIV_DIV1; // CPU clock divided by 1 -> CPU_F = 120MHz
  
	
	SUPC->VREG.bit.SEL = 0; //We are using an LDO regulator.

}

void voltage_ref_setup(){
	
	//Voltage Reference is always on
	SUPC->VREF.bit.ONDEMAND = 0;
	
	//Voltage Reference is set at 1[V]
	SUPC->VREF.bit.SEL = SUPC_VREF_SEL_1V0_Val;
	
	//Vref is routed to an ADC input
	SUPC->VREF.bit.VREFOE = 1;
	
}

void set_up_ADC0(){
	
	GCLK->PCHCTRL[ADC0_GCLK_ID].reg = GENERIC_CLOCK_GENERATOR_48M | //ADC get 48MHz from GenCLK_1
	(1 << GCLK_PCHCTRL_CHEN_Pos); //Enable clock to ADC0
	
	while (!GCLK->PCHCTRL[ADC0_GCLK_ID].bit.CHEN);
	
	//Enable the Main Clock to ADC0
	MCLK->APBDMASK.reg |= MCLK_APBDMASK_ADC0;
	
	//maybe no division ??
	
	//ADC now has 1.5MHz clock
	ADC0->CTRLA.bit.PRESCALER = ADC_CTRLA_PRESCALER_DIV8_Val;
	//ADC0->CTRLA.bit.PRESCALER = ADC_CTRLA_PRESCALER_DIV32_Val;
	
	//ADC resolution set to 10bits
	ADC0->CTRLB.bit.RESSEL = ADC_CTRLB_RESSEL_10BIT_Val;
	
	//ADC0->CTRLB.bit.RESSEL = ADC_CTRLB_RESSEL_12BIT_Val;
	
	//wait for sync
	while( ADC0->SYNCBUSY.reg & ADC_SYNCBUSY_CTRLB ); 
	
	//Sampling Time Control = 5
	//Sampling time = (SAMPLEN+1)*ADC_CLK
	//Maybe not needed since we are not taking an average
	//ADC0->SAMPCTRL.reg = 7500000;
	
	
	//wait for sync
	while( ADC0->SYNCBUSY.reg & ADC_SYNCBUSY_SAMPCTRL );  
	
	//The negative supply of the ADC is connected to GND
	ADC0->INPUTCTRL.reg = ADC_INPUTCTRL_MUXNEG_GND;
	//wait for sync
	while( ADC0->SYNCBUSY.reg & ADC_SYNCBUSY_INPUTCTRL );  
	
	
	ADC0->AVGCTRL.reg = ADC_AVGCTRL_SAMPLENUM_1 | // 1 sample only (no oversampling nor averaging)
	ADC_AVGCTRL_ADJRES(0x1ul);// Division Coefficient set to 0 ?
	
	//wait for sync
	while( ADC0->SYNCBUSY.reg & ADC_SYNCBUSY_AVGCTRL );  
	
	// Set reference to Internal Reference
	//ADC0->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_INTREF; 
	
	// Set reference to VDDANA = 3V3
	ADC0->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_INTVCC1_Val; 
	


	
		
	
}

uint32_t ADC_Read(int phase){
	
	uint32_t valueRead = 0;
	//wait for sync
	while( ADC0->SYNCBUSY.reg & ADC_SYNCBUSY_INPUTCTRL ); 
	//Phase 1 = AIN10
	//Phase 2 = AIN9
	//Phase 3 = AIN8
	
	switch(phase){
		
		case 1:
			ADC0->INPUTCTRL.bit.MUXPOS=ADC_INPUTCTRL_MUXPOS_AIN10;
			break;
		case 2:
			ADC0->INPUTCTRL.bit.MUXPOS=ADC_INPUTCTRL_MUXPOS_AIN9;
			break;
		case 3:
			ADC0->INPUTCTRL.bit.MUXPOS=ADC_INPUTCTRL_MUXPOS_AIN8;
			break;
		default:
			return 0;
	}
	
	while( ADC0->SYNCBUSY.reg & ADC_SYNCBUSY_ENABLE ); //wait for sync
	
	//Enable ADC0
	ADC0->CTRLA.bit.ENABLE=1;
	// Start conversion
	while( ADC0->SYNCBUSY.reg & ADC_SYNCBUSY_ENABLE ); //wait for sync

	//Start the conversation
	ADC0->SWTRIG.bit.START = 1;
	
	// Clear the Data Ready flag since we do not want the first reading
	ADC0->INTFLAG.reg = ADC_INTFLAG_RESRDY;
	
	// Starting the conversion again
	ADC0->SWTRIG.bit.START = 1;
		
	// Waiting for conversion to complete
	while (ADC0->INTFLAG.bit.RESRDY == 0);   
	// Store the value
	valueRead = ADC0->RESULT.reg;	
	
	
	
	//wait for sync
	while( ADC0->SYNCBUSY.reg & ADC_SYNCBUSY_ENABLE ); 
	// Disable ADC
	ADC0->CTRLA.bit.ENABLE = 0x00;             
	 //wait for sync
	while( ADC0->SYNCBUSY.reg & ADC_SYNCBUSY_ENABLE );
	
	
	return valueRead;
	
}

void usb_data_received(uint8_t status, uint16_t length) {
	
	
	memcpy(tmp_buffer,rx_data_buffer,length); // copies data to temporary buffer
	
	
	//Motor Sate Change
	if(tmp_buffer[0] == 0 ){
		
		if(tmp_buffer[1] == 0 ){
			MOTOR_obj.motor_state = DISABLED;
		}else if (tmp_buffer[1] == 1 ){
			MOTOR_obj.motor_state = ENABLED;
		}
	//Motion Type Change	
	}else if (tmp_buffer[0] == 1){
		
		if(tmp_buffer[1] == 0 ){
			motion_control_type = VELOCITY_OPENLOOP;
		}else if (tmp_buffer[1] == 1 ){
			motion_control_type = ANGLE_OPENLOOP;
		}else if (tmp_buffer[1] == 1 ){
			motion_control_type = VELOCITY;
		}else if (tmp_buffer[1] == 1 ){
			motion_control_type = ANGLE;
		}
		
	}else if (tmp_buffer[0] == 2){
	
		FOC_target = tmp_buffer[1];
	
	}
	
	
	//usb_send_bulk_data(tmp_buffer,length,0); // resends data
	usb_receive_bulk_data(); // restarts data reception.
	
}

void set_usb_config(uint8_t index) {
	usb_setup_rx_bulk_buffer((uint8_t*)rx_data_buffer,sizeof(rx_data_buffer),usb_data_received); // sets reception buffer and reception callback
	usb_receive_bulk_data(); // starts data reception
}

void set_up_systick() 
{

	SysTick_Config(F_CPU/1000);// Overflow will occur every 1ms
	//SysTick_Config(F_CPU/1000000);// Overflow will occur every 1ms
	NVIC_SetPriority(SysTick_IRQn, 1);//Set the interrupt has at highest priority
}

void SysTick_Handler(){
	_ulTickCount++;
}

void beep_1KHZ (int milliseconds)
{
  int x = 0;
  //PORTD = B00001000;      //Set D2 (CL) to HIGH and the rest to LOW
  setPhaseVoltage(0,0,0);
  
  while (x < milliseconds)
  { 

	setPhaseVoltage(1,0,0);
    //PORTB = B00000010;      //Set D90 (AH) to HIGH (BH) to LOW
    delayMicroseconds(50);
	setPhaseVoltage(0,0,0);
    //PORTB = B00000000;      //Set D90 (AH) to HIGH (BH) to LOW
	
    delayMicroseconds(450);
    
	setPhaseVoltage(0,1,0);
    //PORTB = B00000100;      //Set D10 (BH) to HIGH (AH) to LOW 
    delayMicroseconds(50);
	setPhaseVoltage(0,0,0);
    //PORTB = B00000000;      //Set D10 (BH) to HIGH (AH) to LOW 
    delayMicroseconds(450);
    x = x + 1;
  }
   //PORTD = B00000000;      //Set D2 (CL) to HIGH and the rest to LOW
   //PORTB = B00000000;      //Set D10 (BH) to HIGH (AH) to LOW 
   
  setPhaseVoltage(0,0,0);
}

void dwt_enable(void){
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; /* Global Enable for DWT */
	DWT->CYCCNT = 0;                                /* Reset the counter */
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;            /* Enable cycle counter */
}

void call_back(){
	
}

int main(void)
{
	memset(buffer,sizeof(buffer[0])*64,0);
	
	open_loop_timestamp = micros();
	velocity_calc_timestamp = micros();
	PINs_setup();
	voltage_ref_setup();
	set_up_PWM_obj();
	set_up_motor_obj();
	set_up_clocks();
	set_up_systick();
	usb_register_set_config_callback(set_usb_config);
	usb_init(GENERIC_CLOCK_GENERATOR_48M);
	//set_TC0();
	set_up_ADC0();
	set_up_current_obj();
	TCC0_setup();
	set_up_SPI_SERCOM_5(10000000); //10MHz
	dwt_enable();
	set_up_LPF();
	set_up_PID();
	//Incase there are any errors in the magnetic sensor
	AS5048A_send_command(AS5048A_CLEAR_ERROR_FLAG);
	setPhaseVoltage(0,0,0);
	delay(1);
	unsigned long current = millis();
	unsigned long previous = millis();
	
	while(sensor_direction == NONE){
		angular_sensor_calibrate();
	}

	RESET(LED4)
	RESET(LED5)
	MOTOR_obj.motor_state = ENABLED;
	motion_control_type = VELOCITY;
	//motion_control_type = VELOCITY_OPENLOOP;
	//motion_control_type = ANGLE;
	//motion_control_type = ANGLE_OPENLOOP;
	
	//rad/sec for Velocity Control
	//rad for Angle Control
	FOC_target = 20;
		
		
	current_sense = DISABLED;

	aprox_trig = FALSE;
	PWM_obj.modulation_type = SINE_PWM;

	while(1){

		FOC(FOC_target);
		


			
		current = millis();
		if (current - previous >= 1000) {
			previous = millis();
			
			enabale_ob = TRUE;
			/*
			if(FOC_target == 100){
				FOC_target = -100;	
			}else{
				FOC_target = 100;		
			}*/
				
		}
		
			
	}
	
	
}

