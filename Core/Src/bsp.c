//============================================================================
// Product: DPP example, NUCLEO-C031C6 board, QK kernel
// Last updated for version 8.0.0
// Last updated on  2024-09-18
//
//                   Q u a n t u m  L e a P s
//                   ------------------------
//                   Modern Embedded Software
//
// Copyright (C) 2005 Quantum Leaps, LLC. <state-machine.com>
//
// This program is open source software: you can redistribute it and/or
// modify it under the terms of the GNU General Public License as published
// by the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// Alternatively, this program may be distributed and modified under the
// terms of Quantum Leaps commercial licenses, which expressly supersede
// the GNU General Public License and are specifically designed for
// licensees interested in retaining the proprietary status of their code.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program. If not, see <www.gnu.org/licenses/>.
//
// Contact information:
// <www.state-machine.com/licensing>
// <info@state-machine.com>
//============================================================================
#include "qpc.h"                 // QP/C real-time event framework
#include "bsp.h"                 // Board Support Package
#include "stm32c0xx.h"  // CMSIS-compliant header file for the MCU used
#include "ssd1306.h"
#include "app_config.h"
#include "main_app.h"
#include "NRF_chip.h"
#include "temp_sensor.h"
#include "stm32c0xx_ll_tim.h"
#include "stm32c0xx_ll_adc.h"
#include "stm32c0xx_ll_spi.h"
#include "stm32c0xx_ll_rtc.h"
#include "stm32c0xx_ll_exti.h"
#include "stm32c0xx_ll_pwr.h"
#include "accelerometer.h"
// add other drivers if necessary...
extern MainApp MainApp_inst;
static void RTC_ReadTime(uint8_t *hours, uint8_t *minutes, uint8_t *seconds);

Q_DEFINE_THIS_FILE  // define the name of this file for assertions

// Local-scope defines -----------------------------------------------------
// LED pins available on the board (just one user LED LD4--Green on PA.5)
#define LD4_PIN  5U

// Button pins available on the board (just one user Button B1 on PC.13)
#define B1_PIN   13U

// Local-scope objects -----------------------------------------------------
static uint32_t l_rndSeed;




#ifdef Q_SPY

    QSTimeCtr QS_tickTime_;
    QSTimeCtr QS_tickPeriod_;

    // QSpy source IDs
    static QSpyId const l_SysTick_Handler = { 0U };
    static QSpyId const l_EXTI0_1_IRQHandler = { 0U };

    enum AppRecords { // application-specific trace records
        PHILO_STAT = QS_USER,
        PAUSED_STAT,
        CONTEXT_SW,
    };

#endif

//============================================================================
// Error handler and ISRs...

Q_NORETURN Q_onError(char const * const module, int_t const id) {
    // NOTE: this implementation of the assertion handler is intended only
    // for debugging and MUST be changed for deployment of the application
    // (assuming that you ship your production code with assertions enabled).
    Q_UNUSED_PAR(module);
    Q_UNUSED_PAR(id);
    QS_ASSERTION(module, id, 10000U);

#ifndef NDEBUG
    // light up the user LED
    //GPIOA->BSRR = (1U << LD4_PIN);  // turn LED on
    // for debugging, hang on in an endless loop...
    for (;;) {
    }
#endif

    NVIC_SystemReset();
}
//............................................................................
// assertion failure handler for the STM32 library, including the startup code
void assert_failed(char const * const module, int_t const id); // prototype
void assert_failed(char const * const module, int_t const id) {
    Q_onError(module, id);
}

// ISRs used in the application ============================================

static volatile uint16_t sysTickCounter = 0;

void SysTick_Handler(void); // prototype
void SysTick_Handler(void) {
    QK_ISR_ENTRY();   // inform QK about entering an ISR
    sysTickCounter++;
    QF_TICK_X(0U, (void *)0);
    //QTIMEEVT_TICK_X(0U, &l_SysTick_Handler); // time events at rate 0

#ifdef Q_SPY
    uint32_t volatile tmp = SysTick->CTRL; // clear CTRL_COUNTFLAG
    QS_tickTime_ += QS_tickPeriod_; // account for the clock rollover
    Q_UNUSED_PAR(tmp);
#endif

    QK_ISR_EXIT();    // inform QK about exiting an ISR
}

void BSP_delayMs(uint32_t ms) {
    uint32_t start = sysTickCounter;
    while ((sysTickCounter - start) < ms);
}



static uint32_t buttonPressTime = 0;
static uint8_t buttonPressed = 0;

void EXTI0_1_IRQHandler(void);
void EXTI0_1_IRQHandler(void)
{
    QK_ISR_ENTRY();


    static QEvt const nrfEvt = { NRF_IRQ_SIG, 0U, 0U };
    QACTIVE_POST(AO_RFButton, &nrfEvt, &l_EXTI0_1_IRQHandler);



    __HAL_GPIO_EXTI_CLEAR_IT(RF_BUTTON_PIN);


    QK_ISR_EXIT();
}


static uint32_t timestamps[TIMESTAMP_SIZE];
static uint8_t pulseCount = 0;
static bool start_dht11 = false;
void TIM3_IRQHandler(void)
{
    QK_ISR_ENTRY();

    // Check for CC1 flag
    if (LL_TIM_IsActiveFlag_CC1(TIM3))
    {
        LL_TIM_ClearFlag_CC1(TIM3);

        // Read the captured value (timer ticks)
		uint32_t capture = LL_TIM_IC_GetCaptureCH1(TIM3);

		if(capture >= 79 && capture <= 90)
		{
			start_dht11 = true;
		}
		if(!start_dht11) //ignore stuff before ACK
		{
			QK_ISR_EXIT();
			return;
		}

		timestamps[pulseCount] = capture;
		pulseCount++;
		//pulseCount %= TIMESTAMP_SIZE;

		if (pulseCount >= TIMESTAMP_SIZE) {
		    static QEvt const dht11CompleteEvt = QEVT_INITIALIZER(DHT11_TIMER_IC_SIG);
		    QACTIVE_POST(AO_Sensor, &dht11CompleteEvt, 0U);
		    pulseCount = 0;
		    start_dht11 = false;
		}
    }

    QK_ISR_EXIT();
}

void BSP_get_timestamps(uint32_t *dest) {
    QF_CRIT_STAT
    QF_CRIT_ENTRY();
    memcpy(dest, timestamps, sizeof(timestamps));
    QF_CRIT_EXIT();
}



void EXTI4_15_IRQHandler(void)
{
	QK_ISR_ENTRY();
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_6);
    QK_ISR_EXIT();
}


void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin)
{
		 if (GPIO_Pin == GPIO_PIN_6) {
		        uint8_t src;
		        //HAL_I2C_Mem_Read(&hi2c1, ADXL_ADDR, ADXL_INT_SOURCE, 1, &src, 1, HAL_MAX_DELAY);
		        HAL_I2C_Mem_Read(&hi2c1, ADXL_ADDR, 0x30, 1, &src, 1, HAL_MAX_DELAY);

		        if (src & 0x10) { // Bit 4 = Activity interrupt

		        }
				if(src & 0x40) // SINGLE_TAP bit
				{
					// Tap detected
					// Example: toggle an LED
					static QEvt const dht11CompleteEvt = QEVT_INITIALIZER(START_TETRIS_SIG);
					QACTIVE_POST(AO_Main_App, &dht11CompleteEvt, 0U);
				}
				//  clear the interrupt
				HAL_I2C_Mem_Read(&hi2c1, ADXL_ADDR, 0x30, 1, &src, 1, HAL_MAX_DELAY);
				__HAL_GPIO_EXTI_CLEAR_IT(GPIO_Pin);

		    }
}


void RTC_IRQHandler(void)
{
    /* Alarm A event occurred? */
    if (LL_RTC_IsActiveFlag_ALRA(RTC))
    {
        /* Clear the Alarm A flag */
        LL_RTC_ClearFlag_ALRA(RTC);

        /* Also clear EXTI line 17 flag */
        LL_EXTI_ClearRisingFlag_0_31(LL_EXTI_LINE_19);

        RTC_setWakeIntervalSeconds(5);
    }
}


//............................................................................


//............................................................................
#ifdef Q_SPY
// ISR for receiving bytes from the QSPY Back-End
// NOTE: This ISR is "QF-unaware" meaning that it does not interact with
// the QF/QK and is not disabled. Such ISRs don't need to call
// QK_ISR_ENTRY/QK_ISR_EXIT and they cannot post or publish events.

void USART2_IRQHandler(void); // prototype
void USART2_IRQHandler(void) { // used in QS-RX (kernel UNAWARE interrutp)
    // is RX register NOT empty?
    if ((USART2->ISR & (1U << 5U)) != 0U) {
        uint32_t b = USART2->RDR;
        QS_RX_PUT(b);
    }

    QK_ARM_ERRATUM_838869();
}
#endif // Q_SPY

//............................................................................
#ifdef QF_ON_CONTEXT_SW
// NOTE: the context-switch callback is called with interrupts DISABLED
void QF_onContextSw(QActive *prev, QActive *next) {
    QS_BEGIN_INCRIT(CONTEXT_SW, 0U) // in critical section!
        QS_OBJ(prev);
        QS_OBJ(next);
    QS_END_INCRIT()
}
#endif // QF_ON_CONTEXT_SW


//============================================================================
// BSP functions...

void BSP_init(void) {
    // Initialize LEDs, buttons, UART, etc.
    //HAL_Init();
    SystemClock_Config();  // configure system clock
    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000U);

	// start TIM17/14
	LL_TIM_EnableCounter(TIM17);
	LL_TIM_EnableCounter(TIM14);
	LL_TIM_EnableCounter(TIM3);

	LL_TIM_SetSlaveMode(TIM3, LL_TIM_SLAVEMODE_RESET);
	LL_TIM_SetTriggerInput(TIM3, LL_TIM_TS_TI1FP1);

	// --- Configure channel 1 for input capture ---
	LL_TIM_IC_SetActiveInput(TIM3, LL_TIM_CHANNEL_CH1, LL_TIM_ACTIVEINPUT_DIRECTTI);
	LL_TIM_IC_SetPolarity(TIM3, LL_TIM_CHANNEL_CH1, LL_TIM_IC_POLARITY_BOTHEDGE);
	LL_TIM_IC_SetPrescaler(TIM3, LL_TIM_CHANNEL_CH1, LL_TIM_ICPSC_DIV1);
	LL_TIM_IC_SetFilter(TIM3, LL_TIM_CHANNEL_CH1, LL_TIM_IC_FILTER_FDIV1);


	NVIC_SetPriority(TIM3_IRQn, QF_AWARE_ISR_CMSIS_PRI);
	NVIC_EnableIRQ(TIM3_IRQn);
	LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH1);
	LL_TIM_EnableCounter(TIM3);          // counter enabled
	LL_TIM_EnableIT_CC1(TIM3);           // interrupt enabled

	//ADC
	LL_ADC_Enable(ADC1);

	ssd1306_Init();
	LL_SPI_Enable(SPI1);
	init_nrf();
	init_accelerometer(); //tap/shake detection
	BSP_RTC_init();
	//init_accelerometer();
	//turn on DHT11
	HAL_GPIO_WritePin(DHT11_RESET_PORT, DHT11_RESET_PIN, GPIO_PIN_SET);
	//__HAL_LINKDMA(&htim3, hdma[TIM_DMA_ID_CC1], hdma_tim3_ch1);

	// Start input capture with DMA
	//HAL_TIM_IC_Start_DMA(&htim3, TIM_CHANNEL_1, (uint32_t *)dht11_dma_buffer, DHT11_MAX_EDGES);
	adc_seed();


	//stop mode:
	RCC->APBENR1 |= RCC_APBENR1_PWREN;
	PWR->CR1 &= ~PWR_CR1_LPMS;
	PWR->SCR |= PWR_SCR_CWUF; //clear wake up flag


}

void BSP_RTC_init(void)
{

	/* Enable RTC interrupt in NVIC */
	NVIC_SetPriority(RTC_IRQn, 3);
	NVIC_EnableIRQ(RTC_IRQn);

	/* Clear any pending RTC Alarm flags */
	LL_RTC_ClearFlag_ALRA(RTC);
	LL_RTC_DisableWriteProtection(RTC);

	/* Enable Alarm A interrupt */
	LL_RTC_EnableIT_ALRA(RTC);

	/* Set first alarm wake in 20 seconds */
	RTC_setWakeIntervalSeconds(10);

	LL_RTC_EnableWriteProtection(RTC);
}

void RTC_setWakeIntervalSeconds(uint32_t seconds)
{
    uint8_t hour, min, sec;

    RTC_ReadTime(&hour, &min, &sec);

    /* Add seconds */
    sec += seconds;
    if (sec >= 60) {
        min += sec / 60;
        sec %= 60;
    }
    if (min >= 60) {
        hour += min / 60;
        min %= 60;
    }
    hour %= 24;

    /* Convert back to BCD */
    uint32_t tr =
	((hour / 10) << RTC_TR_HT_Pos) | ((hour % 10) << RTC_TR_HU_Pos) |
	((min  / 10) << RTC_TR_MNT_Pos) | ((min  % 10) << RTC_TR_MNU_Pos) |
	((sec  / 10) << RTC_TR_ST_Pos)  | ((sec  % 10) << RTC_TR_SU_Pos);

    /* Program Alarm A */
    LL_RTC_DisableWriteProtection(RTC);

    LL_RTC_ALMA_Disable(RTC);
    LL_RTC_WriteReg(RTC, ALRMAR, tr | RTC_ALRMAR_MSK4); // Ignore date
    LL_RTC_ClearFlag_ALRA(RTC);
    LL_RTC_ALMA_Enable(RTC);

    LL_RTC_EnableWriteProtection(RTC);
}



static void RTC_ReadTime(uint8_t *hours, uint8_t *minutes, uint8_t *seconds)
{
    uint32_t tr = LL_RTC_TIME_Get(RTC); // This returns the raw TR register

    uint8_t h1 = (tr >> RTC_TR_HT_Pos) & 0x3;     // tens of hours
    uint8_t h0 = (tr >> RTC_TR_HU_Pos) & 0xF;     // units of hours
    uint8_t m1 = (tr >> RTC_TR_MNT_Pos) & 0x7;    // tens of minutes
    uint8_t m0 = (tr >> RTC_TR_MNU_Pos) & 0xF;    // units of minutes
    uint8_t s1 = (tr >> RTC_TR_ST_Pos) & 0x7;     // tens of seconds
    uint8_t s0 = (tr >> RTC_TR_SU_Pos) & 0xF;     // units of seconds

    *hours   = h1 * 10 + h0;
    *minutes = m1 * 10 + m0;
    *seconds = s1 * 10 + s0;
}


//............................................................................
void BSP_start(void)
{
    // Optional: initialize LEDs or other board indicators
    // BSP_ledInit();

    // Optional: initialize QSPY (if using tracing)
    // QS_initBuf(qsBuf, sizeof(qsBuf));

    // Enter the QP event loop
    (void)QF_run();  // never returns
}
//............................................................................

//............................................................................
static uint16_t rndSeed = 12345;
void BSP_randomSeed(uint32_t seed) {
	rndSeed = seed;
}

void adc_seed(void) {
    uint32_t seed = 0;

    // Start ADC conversion
    LL_ADC_REG_StartConversion(ADC1);

    // Wait for conversion to complete (check EOC flag)
    while (!LL_ADC_IsActiveFlag_EOC(ADC1)) {
        // Timeout protection (optional)
    }

    // Read the value
    seed = LL_ADC_REG_ReadConversionData12(ADC1);

    // Clear the EOC flag
    LL_ADC_ClearFlag_EOC(ADC1);

    rndSeed = seed;
}
//............................................................................


uint32_t BSP_random(uint16_t modulo) {
	rndSeed = rndSeed * 25173 + 13849;
	return (rndSeed >> 8) % modulo;
}

//............................................................................
void BSP_ledOn(void) {
    GPIOA->BSRR = (1U << LD4_PIN);  // turn LED on
}
//............................................................................
void BSP_ledOff(void) {
    GPIOA->BSRR = (1U << (LD4_PIN + 16U));  // turn LED off
}
//............................................................................
void BSP_terminate(int16_t result) {
    Q_UNUSED_PAR(result);
}

//============================================================================
// QF callbacks...
void QF_onStartup(void) {
    // set up the SysTick timer to fire at BSP_TICKS_PER_SEC rate
    SysTick_Config(SystemCoreClock / BSP_TICKS_PER_SEC);

    // assign all priority bits for preemption-prio. and none to sub-prio.
    // NOTE: this might have been changed by STM32Cube.
    NVIC_SetPriorityGrouping(0U);

    // set priorities of ALL ISRs used in the system, see NOTE1
    NVIC_SetPriority(USART2_IRQn,    0U); // kernel UNAWARE interrupt
    NVIC_SetPriority(EXTI0_1_IRQn,   QF_AWARE_ISR_CMSIS_PRI + 0U);
    NVIC_SetPriority(EXTI4_15_IRQn,   QF_AWARE_ISR_CMSIS_PRI + 2U);
    NVIC_SetPriority(SysTick_IRQn,   QF_AWARE_ISR_CMSIS_PRI + 3U);
    // ...

    // enable IRQs...
    NVIC_EnableIRQ(EXTI0_1_IRQn);
    NVIC_EnableIRQ(EXTI4_15_IRQn);

#ifdef Q_SPY
    NVIC_EnableIRQ(USART2_IRQn); // UART2 interrupt used for QS-RX
#endif
}
//............................................................................
void QF_onCleanup(void) {
}

//............................................................................
void QK_onIdle(void) {
	if(currentState != TETRIS)
	{
		// 1. Enable PWR peripheral
		RCC->APBENR1 |= RCC_APBENR1_PWREN;

		// 2. Clear previous wakeup flags
		PWR->SCR |= PWR_SCR_CWUF;

		// 3. Set STOP mode (main regulator)
		PWR->CR1 &= ~PWR_CR1_LPMS;   // LPMS = 0 => STOP mode, main regulator

		// 4. Allow deep sleep
		SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;

		// 5. Enter STOP mode, wait for interrupt
		__WFI();

		SystemClock_Config();
	}




#ifdef Q_SPY
    QS_rxParse();  // parse all the received bytes

    if ((USART2->ISR & (1U << 7U)) != 0U) { // TXE empty?
        QF_INT_DISABLE();
        uint16_t b = QS_getByte();
        QF_INT_ENABLE();

        if (b != QS_EOD) {   // not End-Of-Data?
            USART2->TDR = b; // put into the DR register
        }
    }
#elif defined NDEBUG
    // Put the CPU and peripherals to the low-power mode.
    // you might need to customize the clock management for your application,
    // see the datasheet for your particular Cortex-M MCU.
    __WFI(); // Wait-For-Interrupt
#endif
}

//============================================================================
// QS callbacks...
#ifdef Q_SPY

//............................................................................
static uint16_t const UARTPrescTable[12] = {
    1U, 2U, 4U, 6U, 8U, 10U, 12U, 16U, 32U, 64U, 128U, 256U
};

#define UART_DIV_SAMPLING16(__PCLK__, __BAUD__, __CLOCKPRESCALER__) \
  ((((__PCLK__)/UARTPrescTable[(__CLOCKPRESCALER__)]) \
  + ((__BAUD__)/2U)) / (__BAUD__))

#define UART_PRESCALER_DIV1  0U

// USART2 pins PA.2 and PA.3
#define USART2_TX_PIN 2U
#define USART2_RX_PIN 3U

//............................................................................
uint8_t QS_onStartup(void const *arg) {
    Q_UNUSED_PAR(arg);

    static uint8_t qsTxBuf[2*1024]; // buffer for QS-TX channel
    QS_initBuf(qsTxBuf, sizeof(qsTxBuf));

    static uint8_t qsRxBuf[100];    // buffer for QS-RX channel
    QS_rxInitBuf(qsRxBuf, sizeof(qsRxBuf));

    // enable peripheral clock for USART2
    RCC->IOPENR  |= ( 1U <<  0U);  // Enable GPIOA clock for USART pins
    RCC->APBENR1 |= ( 1U << 17U);  // Enable USART#2 clock

    // Configure PA to USART2_RX, PA to USART2_TX
    GPIOA->AFR[0] &= ~((15U << 4U*USART2_RX_PIN) | (15U << 4U*USART2_TX_PIN));
    GPIOA->AFR[0] |=  (( 1U << 4U*USART2_RX_PIN) | ( 1U << 4U*USART2_TX_PIN));
    GPIOA->MODER  &= ~(( 3U << 2U*USART2_RX_PIN) | ( 3U << 2U*USART2_TX_PIN));
    GPIOA->MODER  |=  (( 2U << 2U*USART2_RX_PIN) | ( 2U << 2U*USART2_TX_PIN));

    // baud rate
    USART2->BRR  = UART_DIV_SAMPLING16(
                       SystemCoreClock, 115200U, UART_PRESCALER_DIV1);
    USART2->CR3  = 0x0000U |      // no flow control
                   (1U << 12U);   // disable overrun detection (OVRDIS)
    USART2->CR2  = 0x0000U;       // 1 stop bit
    USART2->CR1  = ((1U <<  2U) | // enable RX
                    (1U <<  3U) | // enable TX
                    (1U <<  5U) | // enable RX interrupt
                    (0U << 12U) | // 8 data bits
                    (0U << 28U) | // 8 data bits
                    (1U <<  0U)); // enable USART

    QS_tickPeriod_ = SystemCoreClock / BSP_TICKS_PER_SEC;
    QS_tickTime_ = QS_tickPeriod_; // to start the timestamp at zero

    return 1U; // return success
}
//............................................................................
void QS_onCleanup(void) {
}
//............................................................................
QSTimeCtr QS_onGetTime(void) { // NOTE: invoked with interrupts DISABLED
    if ((SysTick->CTRL & 0x00010000U) == 0U) { // not set?
        return QS_tickTime_ - (QSTimeCtr)SysTick->VAL;
    }
    else { // the rollover occurred, but the SysTick_ISR did not run yet
        return QS_tickTime_ + QS_tickPeriod_ - (QSTimeCtr)SysTick->VAL;
    }
}
//............................................................................
// NOTE:
// No critical section in QS_onFlush() to avoid nesting of critical sections
// in case QS_onFlush() is called from Q_onError().
void QS_onFlush(void) {
    for (;;) {
        uint16_t b = QS_getByte();
        if (b != QS_EOD) {
            while ((USART2->ISR & (1U << 7U)) == 0U) { // while TXE not empty
            }
            USART2->TDR = b;
        }
        else {
            break;
        }
    }
}
//............................................................................
void QS_onReset(void) {
    NVIC_SystemReset();
}
//............................................................................
void QS_onCommand(uint8_t cmdId,
                  uint32_t param1, uint32_t param2, uint32_t param3)
{
    Q_UNUSED_PAR(cmdId);
    Q_UNUSED_PAR(param1);
    Q_UNUSED_PAR(param2);
    Q_UNUSED_PAR(param3);
}

#endif // Q_SPY
//----------------------------------------------------------------------------

//============================================================================
// NOTE1:
// The QF_AWARE_ISR_CMSIS_PRI constant from the QF port specifies the highest
// ISR priority that is disabled by the QF framework. The value is suitable
// for the NVIC_SetPriority() CMSIS function.
//
// Only ISRs prioritized at or below the QF_AWARE_ISR_CMSIS_PRI level (i.e.,
// with the numerical values of priorities equal or higher than
// QF_AWARE_ISR_CMSIS_PRI) are allowed to call the QK_ISR_ENTRY/
// QK_ISR_ENTRY macros or any other QF/QK services. These ISRs are
// "QF-aware".
//
// Conversely, any ISRs prioritized above the QF_AWARE_ISR_CMSIS_PRI priority
// level (i.e., with the numerical values of priorities less than
// QF_AWARE_ISR_CMSIS_PRI) are never disabled and are not aware of the kernel.
// Such "QF-unaware" ISRs cannot call ANY QF/QK services. In particular they
// can NOT call the macros QK_ISR_ENTRY/QK_ISR_ENTRY. The only mechanism
// by which a "QF-unaware" ISR can communicate with the QF framework is by
// triggering a "QF-aware" ISR, which can post/publish events.
//
// NOTE2:
// The User LED is used to visualize the idle loop activity. The brightness
// of the LED is proportional to the frequency of the idle loop.
// Please note that the LED is toggled with interrupts locked, so no interrupt
// execution time contributes to the brightness of the User LED.
//
