// Debug defines
//#define SIDESTEP_PIN 6
//#define DEBUG_PRINT

/*
 * Copyright (c) 2021 Jostein Løwer 
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "DmxInput.h"
#include "DmxInput.pio.h"
#if DEBUG_PRINT
#include <stdio.h>
#endif

#if defined(ARDUINO_ARCH_MBED)
  #include <clocks.h>
  #include <irq.h>
  #include <Arduino.h> // REMOVE ME
#else
  #include "pico/time.h"
  #include "hardware/clocks.h"
  #include "hardware/irq.h"
#endif

bool prgm_loaded[] = {false,false};
volatile uint prgm_offsets[] = {0,0};
/*
This array tells the interrupt handler which instance has interrupted.
The interrupt handler has only the ints0 register to go on, so this array needs as many spots as there are DMA channels. 
*/
#define NUM_DMA_CHANS 12
volatile DmxInput *active_inputs[NUM_DMA_CHANS] = {nullptr};

static void startTransfer(volatile DmxInput *instance)
{
    dma_channel_set_write_addr(instance->_dma_chan, instance->_buf, true);
    pio_sm_exec(instance->_pio, instance->_sm, pio_encode_jmp(prgm_offsets[pio_get_index(instance->_pio)]));
    pio_sm_clear_fifos(instance->_pio, instance->_sm);
}

// A frame is in the buffer, _last_packet_length long: start the transfer for the next one,
// stamp the time and tell the owner.
static void frameDone(volatile DmxInput *instance)
{
    startTransfer(instance);

#ifdef ARDUINO
    instance->_last_packet_timestamp = millis();
#else
    instance->_last_packet_timestamp = to_ms_since_boot(get_absolute_time());
#endif
    // Trigger the callback if we have one
    if (instance->_cb != nullptr) {
        (*(instance->_cb))((DmxInput*)instance);
    }

    // Default the packet length to the full length
    instance->_last_packet_length = instance->_num_channels + 1;
}

// The end-of-frame flag a receiver's state machine raises: the program's "irq 1 rel" is
// flag 1 plus the state machine's number, modulo 4, so each receiver on a PIO has its own.
static inline uint frame_irq(uint sm)
{
    return (1u + sm) & 3u;
}

static void dmxinput_pio_irq_handler()
{
    // Every receiver whose own flag is up (several may be, on one PIO)
    for(int i=0;i<NUM_DMA_CHANS;i++) {
        if(active_inputs[i]!=nullptr) {
            volatile DmxInput *instance = active_inputs[i];
            const uint flag = frame_irq(instance->_sm);

            if(!pio_interrupt_get(instance->_pio, flag))
                continue;

            // Capture the transfer count before we abort the dma transfer.
            // Because we're waiting for the interslot delay (about 50uS) it's
            // essentially guaranteed that the DMA has finished transferring
            // all the bytes, so we shouldn't have any race conditions here.
            uint32_t transferCount = (instance->_num_channels + 1) - dma_hw->ch[i].transfer_count;
#if DEBUG_PRINT
            printf("**Packet len = %u\n", transferCount);
#endif
            // Clear interrupt
            pio_interrupt_clear(instance->_pio, flag);

            // Nothing received since the transfer was started: the frame was complete and
            // the DMA handler has already taken it. The transfer it started stays as it is.
            if (transferCount == 0)
                continue;

            instance->_last_packet_length = transferCount;

            // End the transfer and start the next one here. On the RP2040 the abort may
            // raise the channel's completion interrupt (RP2040-E13), and this code used to
            // rely on that to restart the receiver; the RP2350 raises none, so a frame
            // shorter than the buffer left the receiver stopped for good. So the channel's
            // interrupt is off for the abort, and whatever it raised is cleared.
            dma_channel_set_irq0_enabled(i, false);
            // RP2350-E5: the channel must be disabled before it is aborted
            hw_clear_bits(&dma_hw->ch[i].al1_ctrl, DMA_CH0_CTRL_TRIG_EN_BITS);
            dma_channel_abort(i);
            hw_set_bits(&dma_hw->ch[i].al1_ctrl, DMA_CH0_CTRL_TRIG_EN_BITS);
            dma_channel_acknowledge_irq0(i);
            dma_channel_set_irq0_enabled(i, true);

            frameDone(instance);
        }
    }
}

DmxInput::return_code DmxInput::begin(uint pin, uint num_channels, PIO pio)
{
    uint pio_ind = pio_get_index(pio);
    if(!prgm_loaded[pio_ind]) {
        /* 
        Attempt to load the DMX PIO assembly program into the PIO program memory
        */
        if (!pio_can_add_program(pio, &DmxInput_program))
        {
            return ERR_INSUFFICIENT_PRGM_MEM;
        }
        prgm_offsets[pio_ind] = pio_add_program(pio, &DmxInput_program);

        prgm_loaded[pio_ind] = true;
    }

    /* 
    Attempt to claim an unused State Machine into the PIO program memory
    */

    int sm = pio_claim_unused_sm(pio, false);
    if (sm == -1)
    {
        return ERR_NO_SM_AVAILABLE;
    }

    // Set this pin's GPIO function (connect PIO to the pad)
    pio_sm_set_consecutive_pindirs(pio, sm, pin, 1, false);
#if defined(SIDESTEP_PIN)
    // For debugging the PIO code
    pio_sm_set_consecutive_pindirs(pio, sm, SIDESTEP_PIN, 1, true);
    pio_gpio_init(pio, SIDESTEP_PIN);
#endif
    pio_gpio_init(pio, pin);
    gpio_pull_up(pin);

    // Generate the default PIO state machine config provided by pioasm
    pio_sm_config sm_conf;
    sm_conf = DmxInput_program_get_default_config(prgm_offsets[pio_ind]);

    sm_config_set_in_pins(&sm_conf, pin); // for WAIT, IN
    sm_config_set_jmp_pin(&sm_conf, pin); // for JMP

    // Shift to right, autopush disabled
    sm_config_set_in_shift(&sm_conf, true, false, 8);
    // Deeper FIFO as we're not doing any TX
    sm_config_set_fifo_join(&sm_conf, PIO_FIFO_JOIN_RX);

    // Setup the clock divider to run the state machine at exactly 2MHz
    // A fractional divider: 125 MHz / 2 MHz is 62.5, which an integer one would make 62
    // (the receiver 0.8 % fast).
    sm_config_set_clkdiv(&sm_conf, (float)clock_get_hz(clk_sys) / (float)DMX_SM_FREQ_INPUT);

    // Load our configuration, jump to the start of the program and run the State Machine
    pio_sm_init(pio, sm, prgm_offsets[pio_ind], &sm_conf);

#if defined(SIDESTEP_PIN)
    pio_sm_set_sideset_pins(pio, sm, 6);
#endif

    if (pio_ind == 0)
    {
        // set the IRQ handler
        irq_set_exclusive_handler(PIO0_IRQ_0, dmxinput_pio_irq_handler);
        // enable the IRQ
        irq_set_enabled(PIO0_IRQ_0, true);
        // This receiver's end-of-frame flag (frame_irq), next to any other receiver's
        hw_set_bits(&pio0_hw->inte0, PIO_IRQ0_INTE_SM0_BITS << frame_irq(sm));
    }
    else
    {
        // set the IRQ handler
        irq_set_exclusive_handler(PIO1_IRQ_0, dmxinput_pio_irq_handler);
        // enable the IRQ
        irq_set_enabled(PIO1_IRQ_0, true);
        // This receiver's end-of-frame flag (frame_irq), next to any other receiver's
        hw_set_bits(&pio1_hw->inte0, PIO_IRQ0_INTE_SM0_BITS << frame_irq(sm));
    }

    _pio = pio;
    _sm = sm;
    _pin = pin;
    _num_channels = num_channels;
    _buf = nullptr;
    _cb = nullptr;
    // Default for when a full frame has been received
    _last_packet_length = _num_channels + 1;

    _dma_chan = dma_claim_unused_channel(true);

    if (active_inputs[_dma_chan] != nullptr) {
        return ERR_NO_SM_AVAILABLE;
    }
    active_inputs[_dma_chan] = this;

    return SUCCESS;
}

void DmxInput::read(volatile uint8_t *buffer)
{
    if(_buf==nullptr) {
        read_async(buffer);
    }
    unsigned long start = _last_packet_timestamp;
    while(_last_packet_timestamp == start) {
        tight_loop_contents();
    }
}

void dmxinput_dma_handler() {
    for(int i=0;i<NUM_DMA_CHANS;i++) {
        if(active_inputs[i]!=nullptr && (dma_hw->ints0 & (1u<<i))) {
            dma_hw->ints0 = 1u << i;
            volatile DmxInput *instance = active_inputs[i];

#if DEBUG_PRINT
            printf("**Receive frame\n");
#endif
            frameDone(instance);
        }
    }
}

void DmxInput::read_async(volatile uint8_t *buffer, void (*inputUpdatedCallback)(DmxInput*)) {

    _buf = buffer;
    if (inputUpdatedCallback!=nullptr) {
        _cb = inputUpdatedCallback;
    }

    pio_sm_set_enabled(_pio, _sm, false);

    // Reset the PIO state machine to a consistent state. Clear the buffers and registers
    pio_sm_restart(_pio, _sm);

    //setup dma
    dma_channel_config cfg = dma_channel_get_default_config(_dma_chan);

    // Reading from constant address, writing to incrementing byte addresses
    channel_config_set_transfer_data_size(&cfg, DMA_SIZE_8);
    channel_config_set_read_increment(&cfg, false);
    channel_config_set_write_increment(&cfg, true);

    // Pace transfers based on DREQ_PIO0_RX0 (or whichever pio and sm we are using)
    channel_config_set_dreq(&cfg, pio_get_dreq(_pio, _sm, false));

    //channel_config_set_ring(&cfg, true, 5);
    dma_channel_configure(
        _dma_chan, 
        &cfg,
        NULL,    // dst
        &_pio->rxf[_sm],  // src
        DMXINPUT_BUFFER_SIZE(_num_channels),  // transfer count,
        false
    );

    dma_channel_set_irq0_enabled(_dma_chan, true);
    irq_set_exclusive_handler(DMA_IRQ_0, dmxinput_dma_handler);
    irq_set_enabled(DMA_IRQ_0, true);

    //aaand start!
    startTransfer(this);
#ifdef ARDUINO
    _last_packet_timestamp = millis();
#else
    _last_packet_timestamp = to_ms_since_boot(get_absolute_time());
#endif

    pio_sm_set_enabled(_pio, _sm, true);
}

unsigned long DmxInput::latest_packet_timestamp() {
    return _last_packet_timestamp;
}

uint32_t DmxInput::latest_packet_length() {
    return _last_packet_length;
}

uint DmxInput::pin() {
    return _pin;
}

void DmxInput::end()
{
    // Stop the PIO state machine
    pio_sm_set_enabled(_pio, _sm, false);

    // Remove the PIO DMX program from the PIO program memory
    uint pio_id = pio_get_index(_pio);
    bool inuse = false;
    for(uint i=0;i<NUM_DMA_CHANS;i++) {
        if(i==_dma_chan) {
            continue;
        }
        if(pio_id == pio_get_index(active_inputs[i]->_pio)) {
            inuse = true;
            break;
        }
    }
    if(!inuse) {
        prgm_loaded[pio_id] = false;
        pio_remove_program(_pio, &DmxInput_program, prgm_offsets[pio_id]);
        prgm_offsets[pio_id]=0;
    }

    // Unclaim the sm
    pio_sm_unclaim(_pio, _sm);

    dma_channel_unclaim(_dma_chan);
    active_inputs[_dma_chan] = nullptr;

    _buf = nullptr;
}
