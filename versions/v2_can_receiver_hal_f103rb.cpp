#include <Arduino.h>

CAN_HandleTypeDef hcan;

void MX_CAN_Init(void) {
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 9; 
  // ▼▼▼ 다시 Normal로 변경! (중요) ▼▼▼
  hcan.Init.Mode = CAN_MODE_NORMAL;
  // ▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_5TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = ENABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;

  if (HAL_CAN_Init(&hcan) != HAL_OK) {
    Serial.println("❌ CAN Init Fail");
    while (1);
  }

  CAN_FilterTypeDef sFilterConfig;
  sFilterConfig.FilterBank = 0;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterIdHigh = 0x0000;
  sFilterConfig.FilterIdLow = 0x0000;
  sFilterConfig.FilterMaskIdHigh = 0x0000;
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.SlaveStartFilterBank = 14;

  HAL_CAN_ConfigFilter(&hcan, &sFilterConfig);
  HAL_CAN_Start(&hcan);
}

extern "C" void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN1) {
    __HAL_RCC_CAN1_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_AFIO_REMAP_CAN1_2(); 

    // 핀 설정은 그대로 (배선을 바꿨으므로 코드 수정 불필요)
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n=== STM32 Receiver Ready (Normal Mode) ===");
  MX_CAN_Init();
  Serial.println("✅ 대기 중... 배선 반대로 바꿨나요?");
}

void loop() {
  if (HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_RX_FIFO0) > 0) {
    CAN_RxHeaderTypeDef rxHeader;
    uint8_t rxData[8];
    
    if (HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &rxHeader, rxData) == HAL_OK) {
      if (rxHeader.StdId == 0x100 && rxHeader.DLC >= 4) {
         int16_t t_int = (rxData[0] << 8) | rxData[1];
         int16_t s_int = (rxData[2] << 8) | rxData[3];
         float throttle = t_int / 1000.0;
         float steer = s_int / 1000.0;

         Serial.print("[RX] Throttle: ");
         Serial.print(throttle);
         Serial.print(" \t| Steer: ");
         Serial.println(steer);
      } 
    }
  }
}