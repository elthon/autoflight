# autoflight
通过 Autoflight 学些飞控编程。

# 飞控主板
飞控板使用ZIN5（https://item.taobao.com/item.htm?spm=a1z09.2.0.0.1a072e8dkEpTOJ&id=563070934311&_u=i2fpkdtefe9）
外设接口：USART x6、I2C x1、ADC x2、CAN x1、SPI x1、USB x1
遥控输入：PWM捕获 x 8、预留PPM输入接口
控制输出：8路PWM
主控：STM32F407VET6（原理图上是VGT6）
惯导传感器：MPU6050 + AK8975 + SPL06
EEPROM：AT24C02