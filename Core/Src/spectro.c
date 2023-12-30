#include <spectro.h>
#include <buttons.h>
#include <menu.h>
#include <parameters.h>
#include <atomm_utils.h>
#include <spi_sd.h>
#include <stdarg.h>
#include "AMS_AlgoDll.h"
#include "AsterProcessing.h"
#include <shutter.h>
#include "lcd.h"

#define SPECTRO_DEBUG   0
#define SPECTRO_SCREEN  0
#define USE_CALIB       1
#define USE_SERVO       0

#define SIXTEEN_BITS_F 65535.f

#define ARG_PRELOADED   true
#define ARG_NOTLOADED   false

#define TIMEOUT_OCCURED     0xFFFE

#define HEADER_LENGTH   8
/////////////////////////////////////////
// Modifiable Params
//
extern FLOAT    gTmin, gTmax;
extern FLOAT    gOmin, gOmax;
extern FLOAT    gEmin, gEmax;
extern FLOAT    gPmin, gPmax;
extern FLOAT    gOB1, gTB1;
extern FLOAT    gOA1, gTA1;
extern int      gFlg1;
/////////////////////////////////////////


extern uint64_t temp_ticks;
extern char buffer[BUFFER_length];
extern float temperature_inc;
extern int button_status;
extern int menu_status;
extern volatile FLOAT T;
extern volatile uint32_t micro;
volatile uint64_t timeout_ticks=0;
volatile uint32_t timeout_count=0;
volatile uint32_t uart_timeout=UART_TIMEOUT_SETUP;
volatile uint32_t idle_count=0;
volatile uint32_t cycles=0;
extern volatile uint64_t ticks;	
extern parameter param_list[MAX_PARAMS];
extern DAC_params AD420_params;

extern bool shutter_overshoot;

extern gpiopin shutter_detect1;
extern gpiopin shutter_detect2;
extern volatile int angle_deviation;

uint64_t volatile count_UOFlow=0;
uint64_t volatile spectro_ticks=0;
uint32_t volatile ticks_xd=0;
volatile Spectro Qneo;
DAC_data AD420_data;

static volatile FLOAT Oxy;
volatile FLOAT TT;
static volatile FLOAT Em;
volatile uint32_t spectro_timeout=0;
static volatile bool receiving_header=true;

uint32_t volatile Tx7_buffer[TX7_LEN];
uint32_t volatile Rx7_buffer[RX7_LEN];
uint32_t Factory_Calib[RX7_LEN];

volatile bool dma_busy=false;
volatile bool reload_request=false;

volatile static uint16_t last_command;
volatile static uint16_t timeout_command;
volatile static uint16_t bytes_received;
volatile static uint32_t interface_status;
volatile static uint32_t payload_L;
volatile static	uint32_t l_ret[2];



/**
 * @brief Set the up Spectro usart object
 * 
 */
void setup_Spectro_usart(void) {

	gpio_mode_setup(UART_SPECTRO_GPIO, GPIO_MODE_AF, GPIO_PUPD_NONE, RX_PIN|TX_PIN);
	gpio_set_output_options(UART_SPECTRO_GPIO,GPIO_OTYPE_PP, GPIO_OSPEED_25MHZ,TX_PIN);
	gpio_set_af(UART_SPECTRO_GPIO, UART_SPECTRO_AF, RX_PIN|TX_PIN); //set PD8 to UART3_tx

	rcc_periph_clock_enable(RCC_UART_SPECTRO);
    
	usart_set_baudrate(UART_SPECTRO,3000000);
	usart_set_databits(UART_SPECTRO,8);
	usart_set_stopbits(UART_SPECTRO,USART_STOPBITS_1);
	usart_set_mode(UART_SPECTRO,USART_MODE_TX_RX);
	usart_set_parity(UART_SPECTRO,USART_PARITY_NONE);
	usart_set_flow_control(UART_SPECTRO,USART_FLOWCONTROL_NONE);
    usart_enable_tx_dma(UART_SPECTRO);
    usart_enable_rx_dma(UART_SPECTRO);

    
    /*
    usart_set_rx_timeout_value(UART_SPECTRO, 1000000);
    usart_enable_rx_timeout_interrupt(UART_SPECTRO);
    nvic_enable_irq(NVIC_UART7_IRQ);
    usart_enable_rx_timeout(UART_SPECTRO);
    //usart_enable_rx_interrupt(UART_SPECTRO);
    

    USART_CR1(UART_SPECTRO) |= USART_CR1_IDLEIE;
    usart_enable_rx_interrupt(UART_SPECTRO);

    nvic_set_priority(NVIC_UART7_IRQ, 0b10000001);
    nvic_enable_irq(NVIC_UART7_IRQ);
*/
	usart_enable(UART_SPECTRO);
    
}

/**
 * @brief ISR of UART_Spectro
 * 
 */
void uart5_isr(){
    if(usart_get_flag(UART_SPECTRO, USART_ISR_RTOF)){
        //clear flag
        USART_ICR(UART_SPECTRO) |= USART_ICR_RTOCF;
        /*
        dma_disable_stream(DMA1, DMA_STREAM0);
        //disable RX
        USART_CR1(UART_SPECTRO) &= ~USART_CR1_RE;
        
        dma_busy=false;
        */
        //stop_us_count(&cycles);
        timeout_count++;
        //uart_printf("USART_ISR_RTOF \n");


    }
    if(usart_get_flag(UART_SPECTRO, USART_ISR_RXNE)){
        //start_us_count(&cycles);
        idle_count=0;
    }
    if(usart_get_flag(UART_SPECTRO, USART_ISR_IDLE)){
        //clear flag
        USART_ICR(UART_SPECTRO) |= USART_ICR_IDLECF;
        idle_count++;
    }
}

/////////////////////////////////////////////////////////////
//DMA STUFF
//
/////////////////////////////////////////////////////////////

/**
 * @brief 
 * 
 */
void init_uart_dma() {
    rcc_periph_clock_enable(RCC_DMA1);
	/////////////////////////////////////////////////////////
    //UART7_TX : DMA 1 Stream 1 Ch 5
    dma_stream_reset(DMA1, DMA_STREAM7);
	dma_set_priority(DMA1, DMA_STREAM7, DMA_SxCR_PL_VERY_HIGH);
	//UART7 is 8bits
	dma_set_memory_size(DMA1, DMA_STREAM7, DMA_SxCR_MSIZE_32BIT);
	dma_set_peripheral_size(DMA1, DMA_STREAM7, DMA_SxCR_PSIZE_8BIT);
    
	dma_enable_memory_increment_mode(DMA1, DMA_STREAM7);
	dma_set_transfer_mode(DMA1, DMA_STREAM7, 
	DMA_SxCR_DIR_MEM_TO_PERIPHERAL);
	
	dma_set_peripheral_address(DMA1, DMA_STREAM7, (uint32_t) &USART_TDR(UART_SPECTRO));
    dma_set_memory_address(DMA1, DMA_STREAM7, (uint32_t)Tx7_buffer);

    dma_enable_memory_increment_mode(DMA1, DMA_STREAM7);
    dma_disable_peripheral_increment_mode(DMA1, DMA_STREAM7);

	dma_channel_select(DMA1, DMA_STREAM7, DMA_SxCR_CHSEL_4);	
    dma_enable_transfer_complete_interrupt(DMA1, DMA_STREAM7);

    nvic_set_priority(NVIC_DMA1_STREAM7_IRQ, 0b01000001);
    nvic_enable_irq(NVIC_DMA1_STREAM7_IRQ);
    

	/////////////////////////////////////////////////////////
	//Uart7_RX DMA 1 Stream 3 Ch 5
	dma_stream_reset(DMA1, DMA_STREAM0);
	dma_set_priority(DMA1, DMA_STREAM0, DMA_SxCR_PL_HIGH);
	//UART7 is 8bits
	dma_set_memory_size(DMA1, DMA_STREAM0, DMA_SxCR_MSIZE_32BIT);
	dma_set_peripheral_size(DMA1, DMA_STREAM0, DMA_SxCR_PSIZE_8BIT);
    
	dma_enable_memory_increment_mode(DMA1, DMA_STREAM0);
	dma_set_transfer_mode(DMA1, DMA_STREAM0, 
	DMA_SxCR_DIR_PERIPHERAL_TO_MEM);
	
	dma_set_peripheral_address(DMA1, DMA_STREAM0, (uint32_t) &USART_RDR(UART_SPECTRO));
    dma_set_memory_address(DMA1, DMA_STREAM0, (uint32_t)Rx7_buffer);

    dma_enable_memory_increment_mode(DMA1, DMA_STREAM0);
    dma_disable_peripheral_increment_mode(DMA1, DMA_STREAM0);

	dma_channel_select(DMA1, DMA_STREAM0, DMA_SxCR_CHSEL_4);	
    dma_enable_transfer_complete_interrupt(DMA1, DMA_STREAM0);
    
    nvic_set_priority(NVIC_DMA1_STREAM0_IRQ, 0b01000001);
    nvic_enable_irq(NVIC_DMA1_STREAM0_IRQ);
    
	
}


/**
 * @brief 
 * 
 * @param n_words 
 */
void start_uart_tx_transfer(uint32_t n_words){
    
	dma_set_number_of_data(DMA1, DMA_STREAM7, 4*n_words);
    dma_clear_interrupt_flags(DMA1, DMA_STREAM7, DMA_TCIF);
	dma_enable_stream(DMA1, DMA_STREAM7);
}

/**
 * @brief 
 * 
 * @param n_bytes 
 */
void start_uart_rx_reception(uint32_t n_bytes){
    if(Qneo.Default){
        return;
    }
    dma_clear_interrupt_flags(DMA1, DMA_STREAM0, DMA_TCIF);

    if(receiving_header){
        timeout_ticks=ticks+uart_timeout;

        dma_set_number_of_data(DMA1, DMA_STREAM0, n_bytes);
        dma_enable_stream(DMA1, DMA_STREAM0);
        //enable RX
        //USART_CR1(UART_SPECTRO) |= USART_CR1_RE;
    }
    else{
        // This is very important as you can unwillingly read 
        // from the previous byte received if the dma transfer 
        // starts before the new byte is received 
        // while(((USART_ISR(UART_SPECTRO) & USART_ISR_RXNE) == 0) && !Qneo.Default);

        dma_set_number_of_data(DMA1, DMA_STREAM0, n_bytes);
        dma_enable_stream(DMA1, DMA_STREAM0);
    }
}

/**
 * @brief Called when Tx transfer is done
 * 
 */
void  dma1_stream7_isr(){
	if(dma_get_interrupt_flag(DMA1, DMA_STREAM7, DMA_TCIF)){
		dma_clear_interrupt_flags(DMA1, DMA_STREAM7, DMA_TCIF);
        
        start_uart_rx_reception(HEADER_LENGTH);
	}
}

/**
 * @brief Called when Rx transfer is done
 * 
 */
void  dma1_stream0_isr(){
	if(dma_get_interrupt_flag(DMA1, DMA_STREAM0, DMA_TCIF)){
		dma_clear_interrupt_flags(DMA1, DMA_STREAM0, DMA_TCIF);


        if(receiving_header){
            interface_status = Rx7_buffer[0];
            payload_L = Rx7_buffer[1];
            receiving_header = false;

            //print_Spectro_Data(last_command, payload_L);
            
            if(payload_L){ //payload 
                start_uart_rx_reception(payload_L);
            }
            return;
        }
        else{
            //disable RX
            //USART_CR1(UART_SPECTRO) &= ~USART_CR1_RE;

            //start printing & "processing" received data
            //print_Spectro_Data(last_command, payload_L);
        
            dma_busy=false;
            if(spectro_status[0]==SPECTRO_IDLE){
                switch (last_command)
                {
                case SetProcessing_MSG:
                    spectro_status[1]=SPECTRO_START_E;
                    break;
                
                case START_EXPOSURE_MSG:
                    spectro_ticks=ticks;
                    Qneo.Ti_ms=Qneo.AVG*Qneo.Ti/1000+1;
                    spectro_status[1]=SPECTRO_WAITING;
                    break;

                case GetSpectra_MSG:
                    spectro_status[1]=SPECTRO_PROCESS;
                    break;
                
                default:
                    break;
                }
            }
        }

	}
}

/////////////////////////////////////////////////////////////

void uart_SendCommand_dma (uint16_t command, uint16_t nargs, bool Tx_preloaded, ...)
{   
    dma_busy=true;
    last_command=command;
    va_list args;
    va_start(args, nargs);
    uint32_t* temp_arg;

    Tx7_buffer[0]= WRITE_CMD;
    nargs++; nargs*=4;    
    Tx7_buffer[1]= nargs;
    Tx7_buffer[2]= command;

    nargs/=4;
    if(!Tx_preloaded){
        for(size_t x=1; x<nargs; x++)
        {
            temp_arg = va_arg(args, uint32_t*);
            Tx7_buffer[2+x]= *temp_arg;
        }
    }
    //last index is nargs+1 => nargs+2 words to send
    nargs+=2;
    //Receive header 1st
    receiving_header=true;
    //start the dma transfer
    start_uart_tx_transfer(nargs);
}



uint32_t uart_ReadResp32()
{
    uint32_t resp;
    resp=usart_recv_blocking_timeout(UART_SPECTRO);
    resp|=usart_recv_blocking_timeout(UART_SPECTRO)<<8;
    resp|=usart_recv_blocking_timeout(UART_SPECTRO)<<16;
    resp|=usart_recv_blocking_timeout(UART_SPECTRO)<<24;

    return resp;
}

/**
 * @brief Send a command with its arguments to the Qneo.
 * The response is then returned once received (Non blocking)
 * 
 * @param command 
 * @param nargs 
 * @param Tx_preloaded The arguments to be sent are already preloaded to the Tx buffer
 * @param ... Args
 */
void send_receiveSpectro (uint16_t command, uint16_t nargs, bool Tx_preloaded,  ...)
{   

    if(Qneo.Default){
        return;
    }
    va_list args;
    va_start(args, nargs);

	uart_SendCommand_dma(command, nargs, Tx_preloaded, args);

}

uint32_t print_Spectro_Data(uint16_t command_sent, uint16_t p_length){
#if (SPECTRO_DEBUG)	
    uart_printf("CMD sent : 0x%x \n", command_sent);
    uart_printf("INT status : 0x%x \n", interface_status);
    uart_printf("payload L : %d Bytes\n", payload_L);
#endif
if(p_length!=0){
#if (SPECTRO_DEBUG)	
		uart_printf("MSG ret code : 0x%x \n", Rx7_buffer[0]);
#endif
	}

	uint32_t index = p_length%4==0 ? p_length/4 : p_length/4 + 1;
	uint32_t x =0;
	uint32_t idk=0;

	FLOAT value=0., mean_value=0.;
	
    if(index>1){
        //if(index>300) index=300;
        if(command_sent!=GetSpectra_MSG){
            
            for(x=1; x<index; x++){
                
                

                
#if (SPECTRO_DEBUG)	
            if(false){
                idk= (Rx7_buffer[x]<<8)|(Rx7_buffer[x+1]>>24);
                value = *(FLOAT*) &idk;
                uart_printf("DATA %d  : 0x%x - 0x%x - ", x, Rx7_buffer[x], idk);
            }
            else{
                idk=Rx7_buffer[x];
                value = *(FLOAT*) &Rx7_buffer[x];
                uart_printf("DATA %d  : 0x%x - ", x, Rx7_buffer[x]);
            }
                uart_printf("Fval : %f - Uval : %d \n", value, idk);
#endif
            }
        }
        
        else{
            for(x=1; x<=12; x++){

                value = *(FLOAT*) &Rx7_buffer[x];
                
#if (SPECTRO_DEBUG)	
                uart_printf("DATA %d  : 0x%x - ", x, Rx7_buffer[x]);
                uart_printf("Fval : %f - Uval : %d \n", value, Rx7_buffer[x]);
#endif
            }
            FLOAT min_v=INFINITY,max_v=-INFINITY;
            for(x=Spectra_Start_index; x<index; x++){
                value = *(FLOAT*) &Rx7_buffer[x];
                // It's done at Algo level
                //value-= Qneo.Dark[2*(x-Spectra_Start_index)];

                if(value>max_v) max_v=value;
                if(value<min_v) min_v=value;

                mean_value+=fabsf(value);
#if (SPECTRO_DEBUG)	
                uart_printf("%f,", value);
#endif
            }

            mean_value/=index;
            TT=mean_value;

#if (SPECTRO_DEBUG)	
            uart_printf("\nMA Value = %f - Max value = %f - Min value = %f\n", mean_value, max_v, min_v);
#endif
            
#if (SPECTRO_SCREEN)
            ssd1306_Fill(Black);
            ssd1306_SetCursor(2,2);
            ssd1306_WriteString("Mean Value :", Font_7x10, White);

            ssd1306_SetCursor(40,18);
            sprintf(buffer, "%.2f", mean_value);
            ssd1306_WriteString(buffer, Font_7x10, White); //font width * font height

            ssd1306_UpdateScreen();
#endif
        }
    }
}



void Spectro_Setup(){
    while(dma_busy);
#if (SPECTRO_DEBUG)	
    uart_printf("\nRESET_MSG\n");

#endif
    
    send_receiveSpectro(RESET_MSG, 0, ARG_NOTLOADED);

    while(dma_busy);
    if(Qneo.Default){
        return;
    }

#if (SPECTRO_DEBUG)	
    uart_printf("\nGetWavelengths_MSG\n");

#endif

    send_receiveSpectro(GetWavelengths_MSG, 0, ARG_NOTLOADED);

    while(dma_busy);
    if(Qneo.Default){
        return;
    }
    uint32_t PL=payload_L/4;
    for(size_t i=1; i<PL; i++){
        Qneo.Lambdas[i-1]=*(FLOAT*) &Rx7_buffer[i];
    }
    return;
}

void Setup_mesure(Spectro* Sp){
    while(dma_busy);

#if (SPECTRO_DEBUG)	
    uart_printf("\nSetExposureTime_MSG\n");

#endif
    
    send_receiveSpectro(SetExposureTime_MSG, 1, ARG_NOTLOADED, Sp->Ti);

    while(dma_busy);
#if (SPECTRO_DEBUG)	
    uart_printf("\nSetAvgValue_MSG\n");

#endif
    
    send_receiveSpectro(SetAvgValue_MSG, 1, ARG_NOTLOADED, Sp->AVG);

    while(dma_busy);
#if (SPECTRO_DEBUG)	
    uart_printf("\nSetProcessingSteps_MSG\n");

#endif
    send_receiveSpectro(SetProcessing_MSG, 1, ARG_NOTLOADED, Sp->Processing_Steps);
}

void Calibrate_Dark (uint32_t Exposure_time, uint32_t AVG_value){
    
#if(USE_SERVO)
    //Close the optical entrance using the servomotor
    close_shutter();  
#endif
    uint32_t old_Ti=Qneo.Ti;
    //Start Spectro Procedure 
    Qneo.Ti=Exposure_time;
    Qneo.AVG=AVG_value;
    //Set spectro params
    Setup_mesure(&Qneo);

    while(dma_busy);
#if (SPECTRO_DEBUG)	 
    uart_printf("START_EXPOSURE_MSG\n");
#endif    
    send_receiveSpectro(START_EXPOSURE_MSG,  1, ARG_NOTLOADED, (uint32_t)1);


    delay_ms(Exposure_time*AVG_value/1000);
    while(dma_busy);

GET_STATUS:
#if (SPECTRO_DEBUG)	
    uart_printf("GET_STATUS_MSG\n");
#endif
    send_receiveSpectro(GetStatus_MSG, 0, ARG_NOTLOADED);

    while(dma_busy);
    if(Qneo.Default){
        uart_printf("Calib Dark Timeout");
        Qneo.AVG=1;
        return;
    }

    if(Rx7_buffer[0]==MSG_OK && (Rx7_buffer[1]<<24 != MSG_OK)){
        delay_ms(Qneo.Ti*Qneo.AVG/1000);
        goto GET_STATUS;
    }
 #if (SPECTRO_DEBUG)	
    uart_printf("GetSpectra_MSG\n");
#endif
    send_receiveSpectro(GetSpectra_MSG, 0, ARG_NOTLOADED);



    while(dma_busy);
    if(Qneo.Default){
        return;
    }

    if(Rx7_buffer[0]==MSG_OK){
        int PL=payload_L/4;
        for (size_t i=Spectra_Start_index; i<PL; i++){
            Qneo.Dark[i-Spectra_Start_index]=*(FLOAT *) &Rx7_buffer[i];
        }
    }

    upscale_Dark();
    Qneo.Ti = old_Ti;
    Qneo.AVG=1;

#if(USE_SERVO)
    //Open the optical entrance using the servomotor
    open_shutter();
#endif

    uart_printf("Dark Calibrated\n");
}

void test_spectro(){
    //send_receiveSpectro(RESET_MSG,0);
    //send_receiveSpectro(SetExposureTime_MSG, 1, (uint32_t)6000);
    while(dma_busy);
#if (SPECTRO_DEBUG)	
    uart_printf("START_EXPOSURE_MSG\n");
#endif    

    send_receiveSpectro(START_EXPOSURE_MSG,  1, ARG_NOTLOADED, (uint32_t)1);

    delay_ms(Qneo.Ti*Qneo.AVG/1000);

    while(dma_busy);

GET_STATUS:
#if (SPECTRO_DEBUG)	
    uart_printf("GET_STATUS_MSG\n");
#endif
    send_receiveSpectro(GetStatus_MSG, 0, ARG_NOTLOADED);



    while(dma_busy);
    if(Rx7_buffer[0]==MSG_OK && (Rx7_buffer[1]<<24 != MSG_OK)){
        delay_ms(2);
        goto GET_STATUS;
    }
#if (SPECTRO_DEBUG)	
    uart_printf("GetSpectra_MSG\n");
    uart_printf("------------------------------------------------------\n");
#endif

    send_receiveSpectro(GetSpectra_MSG, 0, ARG_NOTLOADED);
    while(dma_busy);
    uint32_t PL=payload_L/4;
    for (size_t i = Spectra_Start_index; i < PL; i++)
    {
        Qneo.SpecPixel_RAW[i-Spectra_Start_index]=*(FLOAT*) &Rx7_buffer[i];
    }
    

}

int Compute_Atomm(){

    //start_us_count(&micro);
    int status = Pred_OET(&Qneo);
    //stop_us_count(&micro);
    //uart_printf("Time taken to compute T : %d cycles or %.4f ms\n\n", micro, (FLOAT)micro/216000);
    if(status){
        if(status==ERR_SATURATION)
            spectro_status[1]=SPECTRO_SATUR;
        else if (status==ERR_OVERFLOW || status==ERR_UNDERFLOW)
            spectro_status[1]=SPECTRO_UO_FLOW;

    }
    return status;
}

int Compute_Aster(){
    int status = Process(&Qneo);
    
    if(status){
        if(status==ERR_SATURATION)
            spectro_status[1]=SPECTRO_SATUR;
        else if (status==ERR_OVERFLOW || status==ERR_UNDERFLOW)
            spectro_status[1]=SPECTRO_UO_FLOW;

    }
    return status;
}

void Upscale_2_256px(FLOAT Table[]){
    for (size_t i = LEN_PIX/2-1; i > 0; i--)
    {
        Table[2*i]=Table[i];
    }
    for (size_t i = 0; i < LEN_PIX/2; i++)
    {
        Table[2*i+1]=(Table[2*i]+Table[2*i+2])/2;
    }
    Table[LEN_PIX-1]=2*Table[LEN_PIX-2]-Table[LEN_PIX-3];    
}

void upscale_Dark(){
    Upscale_2_256px(Qneo.Dark);
}

void upscale_Wavelengths(){
    Upscale_2_256px(Qneo.Lambdas);
    for (size_t i = 0; i < LEN_PIX; i++)
    {
        Qneo.Lambdas[i]*=0.001;
        Qneo.Fdt[i]=.1;
    }
}

void upscale_RAW(){
    Upscale_2_256px(Qneo.SpecPixel_RAW);
}

/**
 * @brief 
 * 
 * @return int 
 */
int Reload_Params(){

// TODO : Add the remaining params eg. Frequency & Shit
    gTmin=param_list[0].value;
    gTmax=param_list[1].value;
    gOmin=param_list[2].value;
    gOmax=param_list[3].value;
    gEmin=param_list[4].value;
    gEmax=param_list[5].value;
    gPmin=param_list[6].value;
    gPmax=param_list[7].value;
    gOB1=param_list[8].value;
    gTB1=param_list[9].value;
    gOA1=param_list[10].value;
    gTA1=param_list[11].value;
    if(param_list[12].value>0.5)
        gFlg1=1;
    else
        gFlg1=0;

    Qneo.Ti_min=param_list[13].value;
    Qneo.Ti_max=param_list[14].value;
    Qneo.MPeriod_ms=param_list[15].value;
    Qneo.CPeriod_ms=param_list[16].value*60*1000;
    Qneo.Calib_TGradient=param_list[17].value;

    bool algo;
    if(param_list[18].value>0.5)
        algo=true;
    else
        algo=false;

    if(algo!=Qneo.Aster && spectro_status[0]!=SPECTRO_BUSY){
        Qneo.Aster=algo;
        hard_fault_handler(); //reset Mcu
    }
    else if(algo!=Qneo.Aster && spectro_status[0]==SPECTRO_BUSY){
        Qneo.Aster=algo;
    }


    

    AD420_params.Temp_range	=gTmax-gTmin;
    AD420_params.Temp_min	=gTmin;
    
    AD420_params.Oxy_range	=param_list[3].value-param_list[2].value;
    AD420_params.Oxy_min	=param_list[2].value;

    AD420_params.Em_range	=param_list[5].value-param_list[4].value;
    AD420_params.Em_min		=param_list[4].value;
    
    return 0;

}

void Request_Reload(){
    reload_request=true;
}

/**
 * @brief convert to 16 bit fixed point 
 * 
 * @param Sp    Spectro
 * @param DAC   DAC Data
 */
void Convert2_16FP(Spectro* Sp ,DAC_data* DAC){
    FLOAT x;

    x=Sp->Temperature-AD420_params.Temp_min;
    if(x<0) x=0;
    x=x/AD420_params.Temp_range*SIXTEEN_BITS_F;

    //x=Sp->Temperature-300;
    //x=x/1100*SIXTEEN_BITS_F;
    DAC->T_FP=(uint16_t) x;

    x=Sp->Oxy-AD420_params.Oxy_min;
    if(x<0) x=0;
    x=x/AD420_params.Oxy_range*SIXTEEN_BITS_F;
    DAC->Oxy_FP=(uint16_t) x;

    x=Sp->Em-AD420_params.Em_min;
    if(x<0) x=0;
    x=x/AD420_params.Em_range*SIXTEEN_BITS_F;
    DAC->Em_FP=(uint16_t) x;

    //convert to big endian
    DAC->T_FP   = (DAC->T_FP << 8) | (DAC->T_FP >>8);
    DAC->Oxy_FP = (DAC->Oxy_FP << 8) | (DAC->Oxy_FP >>8);
    DAC->Em_FP  = (DAC->Em_FP << 8) | (DAC->Em_FP >>8);
}

void Send_Data2DAC(){
    open_spi(DAC_CS_PORT, DAC_CS);
    
    //spi_send(SPI_DAC, AD420_data.T_FP);

    if(Qneo.Aster){
        spi_send(SPI_DAC, 0x0000);
    }else{
        spi_send(SPI_DAC, AD420_data.Oxy_FP);
    }
    
    
    spi_send(SPI_DAC, AD420_data.T_FP);
    
    for (size_t i = 0; i < 2100; i++)
    {
        asm("NOP");
    }
    close_spi(DAC_CS_PORT,DAC_CS);
}

void spectro_thread(){

    switch (spectro_status[0])
    {
    case SPECTRO_BUSY:
        // State where we dont want to make measurements
        break;
    
    case SPECTRO_IDLE:
        if(spectro_status[1]!=SPECTRO_PROCESS && reload_request){
            Reload_Params();
            reload_request=false;
        }
        switch (spectro_status[1]){            
            case SPECTRO_CONFIG: 
                spectro_free=false;
                micro=0;
                
                //start_us_count(&micro);
                // Sends config to the spectro : IT, AVG & Steps
                // All these while_DMA are not needed but
                // are here JUST IN CASE 
                while(dma_busy); 
                ticks_xd=ticks;

                

                Setup_mesure(&Qneo);
                spectro_status[1]=SPECTRO_IDLE;
                break;

            case SPECTRO_START_E:
                
                // Send a sequence of commands to take a number of Spectra 
                while(dma_busy);
                //start_us_count(&micro);
                send_receiveSpectro(START_EXPOSURE_MSG,  1, ARG_NOTLOADED, (uint32_t)1);
                spectro_status[1]=SPECTRO_IDLE;
                break;
            
            case SPECTRO_WAITING:
                spectro_free=true;
                if(ticks-spectro_ticks>Qneo.Ti_ms){
                    spectro_status[1]=SPECTRO_GET_S;
                }
                break;

            case SPECTRO_GET_S:
                spectro_free=false;
                // Send a sequence of commands to take a number of Spectra 
                while(dma_busy);
                send_receiveSpectro(GetStatus_MSG, 0, ARG_NOTLOADED);
                while(dma_busy);
                if(Rx7_buffer[0]==MSG_OK && (Rx7_buffer[1]<<24 != MSG_OK)){
                    spectro_status[1]=SPECTRO_WAITING;
                    spectro_ticks=ticks;
                    Qneo.Ti_ms=2;
                }
                else
                {
                    spectro_status[1]=SPECTRO_ACQUIRE;
                }
                
                break;


            case SPECTRO_ACQUIRE:
                
                // Send a sequence of commands to take a number of Spectra 
                while(dma_busy);
                

                send_receiveSpectro(GetSpectra_MSG,  0, ARG_NOTLOADED);
                spectro_status[1]=SPECTRO_IDLE;
                break;

            case SPECTRO_PROCESS: ; // no declaration after a LABEL in C. 
                
                //get Sensor internal T
                FLOAT CT= *(FLOAT*) &Rx7_buffer[5]; //error: a label can only be part of a statement and a declaration is not a statement
                if (fabs(CT-Qneo.Calib_T) >= Qneo.Calib_TGradient){
                    Qneo.Calib_T=CT;
                    spectro_status[1]=SPECTRO_CALIB;
                    break;
                }

                payload_L/=4;
                for (size_t i = Spectra_Start_index; i < payload_L; i++)
                {
                    Qneo.SpecPixel_RAW[i-Spectra_Start_index]=*(FLOAT*) &Rx7_buffer[i];
                }
                upscale_RAW();
                
                /*
                Qneo.Em=AD420_params.Em_min;
                Qneo.Oxy=AD420_params.Oxy_min;
                Qneo.Temperature=AD420_params.Temp_min;
                */

                // Starts the algo that computes the temperature
                
                start_us_count(&micro);
                int status;

                if(Qneo.Aster){
                    status=Compute_Aster();
                }
                else{
                    status=Compute_Atomm();
                }

                if (spectro_status[1]!=SPECTRO_PROCESS){
                    break;
                }

                count_UOFlow=0;
                
                stop_us_count(&micro);
                ticks_xd=ticks-ticks_xd;
                
                uart_printf("@ %d,%03d s\n", (uint32_t) ticks/1000, (uint32_t) ticks%1000, timeout_count);
                uart_printf("Timeouts : %d - Last timemout CMD : 0x%x\n", timeout_count, timeout_command);
                uart_printf("Pred status : %d\n", status);
                uart_printf("Qneo Ti : %f ms\n", (FLOAT) Qneo.Ti/1000);
                uart_printf("Em : %f - Oxy : %f - Temp :  %f\n", Qneo.Em, Qneo.Oxy,Qneo.Temperature);
                uart_printf("Likelihood : %f \n", Qneo.LikH);
                uart_printf("request time : %d us or %d ms\n", micro/108 ,ticks_xd);
                //uart_printf("Timeout duration : %d\n", cycles);
                //uart_printf("Timeout count : %d\n", timeout_count);
                //uart_printf("IDLE count : %d\n", idle_count);
                spectro_status[1]=SPECTRO_DAC;
                break;
            
            case SPECTRO_DAC:
                // sends the computed Temperature (w/ some other stuff ? yes : Em & Oxy) to the DAC 
                
                Convert2_16FP(&Qneo, &AD420_data);  
                uart_printf("Data 1 sent to DAC : 0x%x\n", AD420_data.T_FP);
                uart_printf("Data 2 sent to DAC : 0x%x\n\n", AD420_data.Oxy_FP);              
                Send_Data2DAC();
                
                spectro_status[1]=SPECTRO_IDLE;

                spectro_free=true;
                break;

            case SPECTRO_CALIB:
                // Spectro stops mesuring and recalibrates it's dark spectrum
                spectro_status[0]=SPECTRO_BUSY;
                Calibrate_Dark(Qneo.Ti, 10);
                spectro_status[0]=SPECTRO_IDLE;
                spectro_status[1]=SPECTRO_IDLE;

                spectro_free=true;
                break;


            case SPECTRO_UO_FLOW:
                if(++count_UOFlow>10){
                    Qneo.Temperature=gTmin;
                    AD420_data.T_FP=0;
                    AD420_data.Oxy_FP=0;
                }
                
                // Request another measure right away ?
                uart_printf("@ %d,%03d s\n", (uint32_t) ticks/1000, (uint32_t) ticks%1000);
                uart_printf("Spectro Under/Overflow !\n\n");

                Send_Data2DAC();

                spectro_status[1]=SPECTRO_IDLE;
                spectro_free=true;
                break;       

            case SPECTRO_SATUR:
                if(++count_UOFlow>10){
                    Qneo.Temperature=gTmin;
                    AD420_data.T_FP=0;
                    AD420_data.Oxy_FP=0;
                }
                // we wait here for next request
                uart_printf("@ %d,%03d s\n", (uint32_t) ticks/1000, (uint32_t) ticks%1000);
                uart_printf("Spectro Saturated !\n\n");

                Send_Data2DAC();

                spectro_status[1]=SPECTRO_IDLE;
                spectro_free=true;
                break;

            
            default:
                break;
        }

        

        break;


    case SPECTRO_DEFAULT:
        Qneo.Default=false;
        timeout_count++;
        timeout_command=last_command;
        uart_printf("-------------------\nSpectro Timeout !\n--------------------\n");
        Display_Spectro(false);
        reset_Qneo();
        

#if (SPECTRO_DEBUG)	
        uart_printf("\nRESET_MSG\n");
#endif
        uart_timeout=UART_TIMEOUT_SETUP;
        send_receiveSpectro(RESET_MSG, 0, ARG_NOTLOADED);
        
        while(dma_busy);

        if(Qneo.Default){
            break;
        }

        uart_timeout=UART_TIMEOUT_POST;

        spectro_status[0]=SPECTRO_IDLE;
        spectro_status[1]=SPECTRO_IDLE;
        Display_Spectro(true);
        break;


    default:
        break;
    }
    if(spectro_free){
        menu_thread();  //thread for managing menu navigation/actions
    }
}


