

#include "main.h"
extern TIM_HandleTypeDef htim7;
// �ӳٺ������ӳ�ָ��΢��
void delay_us(uint16_t us) {
  __HAL_TIM_SET_COUNTER(&htim7, 0); // ���ö�ʱ����������ʼֵΪ 0
  HAL_TIM_Base_Start(&htim7);       // ������ʱ��
  while (__HAL_TIM_GET_COUNTER(&htim7) <us) { // �ȴ���ʱ���������ﵽָ�����ӳ�ֵ
    // ��ѯֱ��������ֵ�ﵽ�趨��΢����
  }
  HAL_TIM_Base_Stop(&htim7); // ֹͣ��ʱ��
}