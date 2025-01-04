/*
 * CanDriver.cpp
 *
 *  Created on: Dec 28, 2024
 *      Author: emrullah
 */

#include "CanDriver.h"

// Static map initialization
const std::map<CAN_TypeDef*, std::vector<CanDriver::CanPinConfig>> CanDriver::canPinMap = {
  {CAN1, {
    {GPIOA, GPIO_PIN_11, GPIO_AF9_CAN1}, // RX
    {GPIOA, GPIO_PIN_12, GPIO_AF9_CAN1}  // TX
  }},
  {CAN2, {
    {GPIOB, GPIO_PIN_12, GPIO_AF9_CAN2}, // RX
    {GPIOB, GPIO_PIN_13, GPIO_AF9_CAN2}  // TX
  }}
};

CanDriver::CanDriver(CAN_TypeDef* canInstance, Mode mode, uint32_t baudRate)
  : _canInstance(canInstance), _mode(mode), _baudRate(baudRate) {

  enableClock();
  configurePins();
}

void CanDriver::enableClock() {
  if      (_canInstance == CAN1) __HAL_RCC_CAN1_CLK_ENABLE();
  else if (_canInstance == CAN2) __HAL_RCC_CAN2_CLK_ENABLE();
  else return; // Unsupported CAN instance
}

void CanDriver::configurePins() {
  if (canPinMap.find(_canInstance) == canPinMap.end()) {
    return; // No pins available for this CAN instance
  }

  for (const auto& pinConfig : canPinMap.at(_canInstance)) {
    GPIODevice gpioDevice(pinConfig.port, pinConfig.pin);
    gpioDevice.configurePin(
        GPIODevice::Mode::AlternateFunction,
        GPIODevice::OutputType::PushPull,
        GPIODevice::Speed::High,
        GPIODevice::Pull::NoPull,
        pinConfig.alternateFunction
    );
  }
}

void CanDriver::configureCAN(uint32_t prescaler, Mode mode, uint32_t sjw, uint32_t bs1, uint32_t bs2) {
  // Request initialization mode
  _canInstance->MCR |= CAN_MCR_INRQ;
  while ((_canInstance->MSR & CAN_MSR_INAK) == 0);

  // Enable CAN peripheral
  _canInstance->MCR &= ~CAN_MCR_SLEEP;
  while ((_canInstance->MSR & CAN_MSR_SLAK)!=0U);

  // Reset BTR configuration
  _canInstance->BTR = 0;

  // Configure CAN bit timing register
  _canInstance->BTR |= ((sjw - 1) << CAN_BTR_SJW_Pos) |
                       ((bs1 - 1) << CAN_BTR_TS1_Pos) |
                       ((bs2 - 1) << CAN_BTR_TS2_Pos) |
                       ((prescaler - 1) << CAN_BTR_BRP_Pos) |
                       (static_cast<uint32_t>(mode) << CAN_BTR_LBKM_Pos);
}

uint32_t CanDriver::encode32BitFilter(uint32_t value) const {
  return (value & 0x1FFFFFFF) << 3; // Mask to 29 bits and shift
}

// Helper function to encode a 16-bit filter value (split into two halves)
std::pair<uint32_t, uint32_t> CanDriver::encode16BitFilter(uint32_t id, uint32_t mask) const {
  uint32_t fr1 = ((id & 0x7FF) << 5) | ((mask & 0x7FF) << 21); // Lower halves
  uint32_t fr2 = (((id >> 11) & 0x7FF) << 5) | (((mask >> 11) & 0x7FF) << 21); // Upper halves
  return {fr1, fr2};
}

void CanDriver::configureFilter(uint8_t filterBank, uint8_t startBank, uint32_t id, uint32_t mask,
                                uint8_t fifo, bool isIdentifierList, bool is32Bit) {
  // Enable filter initialization mode
  _canInstance->FMR |= CAN_FMR_FINIT;

  // Configure the filter bank start for CAN2
  if (_canInstance == CAN2) {
    _canInstance->FMR &= ~(CAN_FMR_CAN2SB_Msk);
    _canInstance->FMR |= (startBank << CAN_FMR_CAN2SB_Pos);
  }

  // Configure the filter scale
  if (is32Bit) {
    _canInstance->FS1R |= (1U << filterBank); // Set filter scale to 32-bit
  } else {
    _canInstance->FS1R &= ~(1U << filterBank); // Set filter scale to 16-bit
  }

  // Configure the identifier mode
  if (isIdentifierList) {
    _canInstance->FM1R |= (1U << filterBank); // Identifier list mode
  } else {
    _canInstance->FM1R &= ~(1U << filterBank); // Identifier mask mode
  }

  // Assign the filter to the selected FIFO
  if (fifo == 1) {
    _canInstance->FFA1R |= (1U << filterBank); // Assign to FIFO 1
  } else {
    _canInstance->FFA1R &= ~(1U << filterBank); // Assign to FIFO 0
  }

  // Deactivate the filter bank before configuring it
  _canInstance->FA1R &= ~(1U << filterBank);

  // Configure the filter's identifier and mask
  if (is32Bit) {
    _canInstance->sFilterRegister[filterBank].FR1 = encode32BitFilter(id);
    _canInstance->sFilterRegister[filterBank].FR2 = encode32BitFilter(mask);
  } else {
    auto [fr1, fr2] = encode16BitFilter(id, mask);
    _canInstance->sFilterRegister[filterBank].FR1 = fr1;
    _canInstance->sFilterRegister[filterBank].FR2 = fr2;
  }

  // Activate the filter
  _canInstance->FA1R |= (1U << filterBank);

  // Exit filter initialization mode
  _canInstance->FMR &= ~CAN_FMR_FINIT;
}

void CanDriver::start() {
  // Leave Initialization mode
  _canInstance->MCR &= ~CAN_MCR_INRQ;
  while (_canInstance->MSR & CAN_MSR_INAK);
}

void CanDriver::sendMessage(TxFrame *frame )
{
  // Wait until the transmit mailbox is empty
  while ((_canInstance->TSR & CAN_TSR_TME0) == 0);

  // Set the standard identifier and data length
  _canInstance->sTxMailBox[0].TIR &= ~CAN_TI0R_STID;

  // Configure the transmit mailbox identifier
  _canInstance->sTxMailBox[0].TIR |= (frame->identifier << 21);

  // Configure data length
  _canInstance->sTxMailBox[0].TDTR |= (frame->length << 0);

  _canInstance->sTxMailBox[0].TDLR=
          ((uint32_t)frame->data[3] << CAN_TDL0R_DATA3_Pos) |
          ((uint32_t)frame->data[2] << CAN_TDL0R_DATA2_Pos) |
          ((uint32_t)frame->data[1] << CAN_TDL0R_DATA1_Pos) |
          ((uint32_t)frame->data[0] << CAN_TDL0R_DATA0_Pos);

  _canInstance->sTxMailBox[0].TDHR=
          ((uint32_t)frame->data[7] << CAN_TDH0R_DATA7_Pos) |
          ((uint32_t)frame->data[6] << CAN_TDH0R_DATA6_Pos) |
          ((uint32_t)frame->data[5] << CAN_TDH0R_DATA5_Pos) |
          ((uint32_t)frame->data[4] << CAN_TDH0R_DATA4_Pos);

  // Set the TXRQ bit to request transmission
  _canInstance->sTxMailBox[0].TIR |= CAN_TI0R_TXRQ;
}

void CanDriver::receiveMessage(RxFrame *frame)
{
  if (_canInstance->RF0R & CAN_RF0R_FMP0) { // Check if there's a pending message in FIFO0
    // Read the received identifier
    frame->identifier = (_canInstance->sFIFOMailBox[0].RIR >> 3) & 0x1FFFFFFF;

    // Read the data length
    frame->length = _canInstance->sFIFOMailBox[0].RDTR & 0x0F;

    /*Clear old data*/
    for (int i=0;i<8;i++)
      frame->data[i]=0;

    frame->data[0]=_canInstance->sFIFOMailBox[0].RDLR >>CAN_RDL0R_DATA0_Pos;
    frame->data[1]=_canInstance->sFIFOMailBox[0].RDLR >>CAN_RDL0R_DATA1_Pos;
    frame->data[2]=_canInstance->sFIFOMailBox[0].RDLR >>CAN_RDL0R_DATA2_Pos;
    frame->data[3]=_canInstance->sFIFOMailBox[0].RDLR >>CAN_RDL0R_DATA3_Pos;

    frame->data[4]=_canInstance->sFIFOMailBox[0].RDHR >>CAN_RDH0R_DATA4_Pos;
    frame->data[5]=_canInstance->sFIFOMailBox[0].RDHR >>CAN_RDH0R_DATA5_Pos;
    frame->data[6]=_canInstance->sFIFOMailBox[0].RDHR >>CAN_RDH0R_DATA6_Pos;
    frame->data[7]=_canInstance->sFIFOMailBox[0].RDHR >>CAN_RDH0R_DATA7_Pos;

    // Release the FIFO (not necessary for FIFO0)
    _canInstance->RF0R |= CAN_RF0R_RFOM0;

    GPIOA->ODR^=GPIO_ODR_OD5;
  }
}
