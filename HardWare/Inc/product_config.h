#ifndef PRODUCT_CONFIG_H
#define PRODUCT_CONFIG_H
//定义产品型号
#define PRODUCT_MODEL_IS_9C_U_HTP100S            0U
#define PRODUCT_MODEL_IS_9C_U_HTP100             1U

#if (PRODUCT_MODEL_IS_9C_U_HTP100S + PRODUCT_MODEL_IS_9C_U_HTP100) != 1U
#error "Select exactly one product model"
#endif

#if PRODUCT_MODEL_IS_9C_U_HTP100S
#define PRODUCT_MODEL_9C_U_HTP100S
#define PRODUCT_MODEL_NAME                       "9C-U-HTP100S"
#elif PRODUCT_MODEL_IS_9C_U_HTP100
#define PRODUCT_MODEL_9C_U_HTP100
#define PRODUCT_MODEL_NAME                       "9C-U-HTP100"
#endif
//定义产品编译日期
#define SOFTWARE_VERSION                         20260702U
//眼盾是否销毁
#define PRODUCT_ENABLE_EYE_SHIELD_MARK           1U

#endif
