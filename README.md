# 小米电机

- [文档](http://roboticscv.com/wp-content/uploads/pdf/CyberGear微电机使用说明书.pdf)
- [资料](https://github.com/belovictor/cybergear-docs)

python 代码：

- [Cybergear](https://github.com/Tony607/Cybergear)


串口助手：[SuperCom](https://github.com/SuperStudio/SuperCom)

## 指令编码

[小米CyberGear微电机串口控制指令驱动](https://blog.csdn.net/weixin_41702812/article/details/136517271)

由于使用的是[YourCee的USB-CAN模块](https://wit-motion.yuque.com/wumwnr/docs/nrngkq)，根据`CAN_ID(报文标识符)说明`进行编码

## 数据编码

```mermaid
flowchart LR
USB2CAN[USB转CAN] 
PC[电脑]
motor[电机]
PC --"USB_Data"--> USB2CAN --"Mi_Data"--> motor
```

USB-CAN模块采用的是 [YourCee](https://wit-motion.yuque.com/wumwnr/docs/nrngkq)

Mi_Data 说明书上有，基本格式为(29 bits + 8 bytes)的格式

| **数据域** | **29 位 ID** | **29 位 ID** | **29 位 ID** | **8Byte 数据区** |
|------------|--------------|--------------|--------------|------------------|
| **大小**   | bit28~bit24  | bit23~8      | bit7~0       | Byte0~Byte7      |
| **描述**   | 通信类型     | 数据区 2     | 目标地址     | 数据区 1         |

由于采用的是USB-CAN模块, 所以需要考虑USB_Data 到 Mi_Data 的转换

根据[YourCee](https://wit-motion.yuque.com/wumwnr/docs/nrngkq)的`CAN_ID(报文标识符)说明`，ID的关系是这样的

$$
\text{USB-id}(\underbrace{\text{id}}_{\text{高 29 bits}} + 100) \rightarrow \text{Mi-id}
$$



AT 模式的 USB 格式:

$$
AT + \text{USB-id} + \underbrace{08}_{\text{1 byte}} + \text{8 bytes} + \textbackslash r\textbackslash n
$$
