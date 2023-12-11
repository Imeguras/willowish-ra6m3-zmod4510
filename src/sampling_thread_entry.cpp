#include "sampling_thread.h"
typedef enum
{
   red = 1,
   green = 2,
   blue = 3,
} led_state_t;
extern bsp_leds_t g_bsp_leds;

static rm_zmod4xxx_oaq_2nd_data_t gs_oaq_2nd_gen_data;
extern          TX_THREAD                  sampling_thread;
void g_comms_i2c_bus0_quick_setup(TX_THREAD* thread_ptr);
void zmod4xxx_sensor0_comms_i2c_callback(rm_zmod4xxx_callback_args_t * p_args);
void zmod4xxx_sensor0_irq_callback(rm_zmod4xxx_callback_args_t * p_args);
typedef enum e_demo_callback_status
{
    DEMO_CALLBACK_STATUS_WAIT = (0),
    DEMO_CALLBACK_STATUS_SUCCESS,
    DEMO_CALLBACK_STATUS_REPEAT,
    DEMO_CALLBACK_STATUS_DEVICE_ERROR
} demo_callback_status_t;
bool g_comms_i2c_bus0_setup = false;
volatile demo_callback_status_t            gs_i2c_callback_status = DEMO_CALLBACK_STATUS_WAIT;

void led_update(led_state_t led_state, e_bsp_io_level value);
/* Sampling Thread entry function */
void sampling_thread_entry(void) {
	float humidity = 70;
	float temperature = 20.7;
	led_update(red, BSP_IO_LEVEL_HIGH);
	   fsp_err_t               err;
	    st_rm_zmod4xxx_raw_data raw_data;

	    /* Clear status */
	    gs_i2c_callback_status = DEMO_CALLBACK_STATUS_WAIT;
	#if G_ZMOD4XXX_SENSOR1_IRQ_ENABLE
	    gs_irq_callback_status = DEMO_CALLBACK_STATUS_WAIT;
	#endif

	if(!g_comms_i2c_bus0_setup)   {
		/* Open the Bus */
		g_comms_i2c_bus0_quick_setup(&sampling_thread);
	}

	g_zmod4xxx_sensor0.p_api->open(g_zmod4xxx_sensor0.p_ctrl, g_zmod4xxx_sensor0.p_cfg);
	
	led_update(red, BSP_IO_LEVEL_LOW);
	
	bool stabilization=false;
	for(int i =0; i <901; i++){
		g_zmod4xxx_sensor0.p_api->measurementStart(g_zmod4xxx_sensor0.p_ctrl);
		
		R_BSP_SoftwareDelay(2,BSP_DELAY_UNITS_SECONDS);
		g_zmod4xxx_sensor0.p_api->read(g_zmod4xxx_sensor0.p_ctrl, &raw_data);
		g_zmod4xxx_sensor0.p_api->temperatureAndHumiditySet(g_zmod4xxx_sensor0.p_ctrl, temperature, humidity);
		g_zmod4xxx_sensor0.p_api->measurementStop(g_zmod4xxx_sensor0.p_ctrl);
		auto k = g_zmod4xxx_sensor0.p_api->oaq2ndGenDataCalculate(g_zmod4xxx_sensor0.p_ctrl, &raw_data, &gs_oaq_2nd_gen_data);

		if (k == FSP_ERR_SENSOR_IN_STABILIZATION ){
			i=0;
			if(stabilization ==false){
				stabilization = true;
				led_update(red, BSP_IO_LEVEL_HIGH);

			}


		}else if(k == FSP_SUCCESS && stabilization ==true){
			stabilization = false;
			led_update(red, BSP_IO_LEVEL_LOW);
			//whatever comes first
			break;
		}
		
	}
	    /* Open ZMOD4XXX */




	//led_update(red, BSP_IO_LEVEL_LOW);
    //led_update(green, BSP_IO_LEVEL_HIGH);
	led_update(blue, BSP_IO_LEVEL_HIGH);

	while (1) {
		//TODO shitty code should be function but at this point i dont care
		g_zmod4xxx_sensor0.p_api->measurementStart(g_zmod4xxx_sensor0.p_ctrl);
		//TODO: use processor timer instead of an actual delay as we can use time stuck post'ing as wait time for sensor
		R_BSP_SoftwareDelay(2,BSP_DELAY_UNITS_SECONDS);
		g_zmod4xxx_sensor0.p_api->read(g_zmod4xxx_sensor0.p_ctrl, &raw_data);
		g_zmod4xxx_sensor0.p_api->temperatureAndHumiditySet(g_zmod4xxx_sensor0.p_ctrl, temperature, humidity);
		g_zmod4xxx_sensor0.p_api->measurementStop(g_zmod4xxx_sensor0.p_ctrl);
		g_zmod4xxx_sensor0.p_api->oaq2ndGenDataCalculate(g_zmod4xxx_sensor0.p_ctrl, &raw_data, &gs_oaq_2nd_gen_data);

	}
}
void g_comms_i2c_bus0_quick_setup(TX_THREAD* thread_ptr)
{
    fsp_err_t err;
    i2c_master_instance_t * p_driver_instance = (i2c_master_instance_t *) g_comms_i2c_bus0_extended_cfg.p_driver_instance;

    /* Open I2C driver, this must be done before calling any COMMS API */
    err = p_driver_instance->p_api->open(p_driver_instance->p_ctrl, p_driver_instance->p_cfg);
    if (FSP_SUCCESS != err)
    {
        tx_thread_delete(thread_ptr);
    }

    /* Create a semaphore for blocking if a semaphore is not NULL */
    if (NULL != g_comms_i2c_bus0_extended_cfg.p_blocking_semaphore)
    {
        tx_semaphore_create(g_comms_i2c_bus0_extended_cfg.p_blocking_semaphore->p_semaphore_handle,
                            g_comms_i2c_bus0_extended_cfg.p_blocking_semaphore->p_semaphore_name,
                            (ULONG) 0);
    }

    /* Create a recursive mutex for bus lock if a recursive mutex is not NULL */
    if (NULL != g_comms_i2c_bus0_extended_cfg.p_bus_recursive_mutex)
    {
        tx_mutex_create(g_comms_i2c_bus0_extended_cfg.p_bus_recursive_mutex->p_mutex_handle,
                        g_comms_i2c_bus0_extended_cfg.p_bus_recursive_mutex->p_mutex_name,
                        TX_INHERIT);
    }

    /* Set setup flag */
    g_comms_i2c_bus0_setup = true;
}
void zmod4xxx_sensor0_irq_callback(rm_zmod4xxx_callback_args_t * p_args)
{
#if G_ZMOD4XXX_SENSOR1_IRQ_ENABLE
    FSP_PARAMETER_NOT_USED(p_args);

    gs_irq_callback_status = DEMO_CALLBACK_STATUS_SUCCESS;
#else
    FSP_PARAMETER_NOT_USED(p_args);
#endif
}

 void led_update(led_state_t led_state, e_bsp_io_level value){
	//BSP_IO_LEVEL_LOW
	//BSP_IO_LEVEL_HIGH
   R_BSP_PinAccessEnable();
    switch(led_state){
        case red:
        {

            R_IOPORT_PinWrite(&g_ioport_ctrl, (bsp_io_port_pin_t) g_bsp_leds.p_leds[2], value);
            break;
        }
        case green:
        {

            R_IOPORT_PinWrite(&g_ioport_ctrl, (bsp_io_port_pin_t) g_bsp_leds.p_leds[1], value);

            break;
        }
        case blue:
        {
            /* Blue LED state is made high to show operation is in progress */
            R_IOPORT_PinWrite(&g_ioport_ctrl, (bsp_io_port_pin_t) g_bsp_leds.p_leds[0], value);
            break;
        }
        default:
        {
            break;
        }
    }
    R_BSP_PinAccessDisable();
}
 void zmod4xxx_sensor0_comms_i2c_callback(rm_zmod4xxx_callback_args_t * p_args)
 {
     if ((RM_ZMOD4XXX_EVENT_DEV_ERR_POWER_ON_RESET == p_args->event)
      || (RM_ZMOD4XXX_EVENT_DEV_ERR_ACCESS_CONFLICT == p_args->event))
     {
         gs_i2c_callback_status = DEMO_CALLBACK_STATUS_DEVICE_ERROR;
     }
     else if (RM_ZMOD4XXX_EVENT_ERROR == p_args->event)
     {
         gs_i2c_callback_status = DEMO_CALLBACK_STATUS_REPEAT;
     }
     else
     {
         gs_i2c_callback_status = DEMO_CALLBACK_STATUS_SUCCESS;
     }
 }


