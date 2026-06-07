#include "AS5047P_RW.h"
#include "stdint.h"
#include "sys.h"

/* 对 AS5047P 底层 SPI 读写做局部 O2 优化，降低角度读取耗时 */
#if defined(__GNUC__) && !defined(__clang__)
#define AS5047P_OPTIMIZE __attribute__((optimize("O2,fast-math")))
#else
#define AS5047P_OPTIMIZE
#endif

/* AS5047P 常用命令 */
#define AS5047P_CMD_READ_ANGLEUNC  0x7FFEU   /* 读未补偿角度 ANGLEUNC */
#define AS5047P_CMD_READ_NOP       0xC000U   /* NOP，用于 pipeline 第二帧取回数据 */

#define AS5047P_READ_RETRY_MAX     3U
#define AS5047P_ENABLE_FRAME_PARITY_CHECK 0U


/* 启用 DWT 周期计数器，用于 us 级超时判断 */
static inline void as5047p_enable_dwt_cycle_counter(void)
{
    if ((CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk) == 0U) {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    }

    if ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) == 0U) {
        DWT->CYCCNT = 0U;
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    }
}


/* AS5047P 帧校验：低 15 位偶校验 */
static inline uint8_t as5047p_calc_parity(uint16_t value)
{
    value &= 0x7FFFU;
    value ^= value >> 8;
    value ^= value >> 4;
    value ^= value >> 2;
    value ^= value >> 1;
    return value & 1U;
}


/* 生成 AS5047P 读寄存器命令：
 * bit14 = 1 表示 read，bit15 为偶校验位。
 */
static inline uint16_t as5047p_make_read_cmd(uint16_t reg)
{
    uint16_t cmd = reg | 0x4000U;

    if (as5047p_calc_parity(cmd)) {
        cmd |= 0x8000U;
    }

    return cmd;
}


/* CS 拉高后的最小间隔延时。
 * 这里用 NOP 保证两次 SPI transaction 之间满足 tCSH。
 */
static inline void as5047p_tCSH_delay(void)
{
    __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
    __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
    __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
    __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
    __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
    __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
}


/* SPI 异常恢复：
 * 清 RX FIFO、清 OVR/FRE/MODF，并确保 SPI 重新使能。
 * 这个函数只在检测到异常或超时时调用，不走正常高速路径。
 */
static inline void as5047p_spi_recover(AS5047P_Handle_t *dev)
{
    SPI_TypeDef *spi = dev->hspi->Instance;

    while ((spi->SR & SPI_SR_FRLVL) != SPI_FRLVL_EMPTY) {
        (void)*(__IO uint16_t *)&spi->DR;
    }

    if (spi->SR & SPI_SR_OVR) {
        (void)*(__IO uint16_t *)&spi->DR;
        (void)spi->SR;
    }

    if (spi->SR & SPI_SR_FRE) {
        (void)spi->SR;
    }

    if (spi->SR & SPI_SR_MODF) {
        (void)spi->SR;
        CLEAR_BIT(spi->CR1, SPI_CR1_SPE);
        SET_BIT(spi->CR1, SPI_CR1_SPE);
    }

    if ((spi->CR1 & SPI_CR1_SPE) == 0U) {
        SET_BIT(spi->CR1, SPI_CR1_SPE);
    }
}


/* 单次 16bit SPI 收发。
 * 直接操作 SPI 寄存器，避免 HAL 阻塞调用开销。
 * 正常路径：等待 TXE -> 写 DR -> 等 RXNE -> 读 DR -> 等 BSY 清零。
 */
AS5047P_OPTIMIZE
static AS5047P_Status_t as5047p_spi_transfer16(AS5047P_Handle_t *dev,
                                               uint16_t tx,
                                               uint16_t *rx)
{
    if (!dev || !dev->hspi || !rx) return AS5047P_ERROR;

    SPI_TypeDef *spi = dev->hspi->Instance;
    const uint32_t TO = 3400U; /* 约 20us @170MHz */
    uint32_t t0;
    uint32_t sr;

    /* 进入传输前先检查 SPI 状态，避免带病通信 */
    sr = spi->SR;
    if ((sr & (SPI_SR_OVR | SPI_SR_FRE | SPI_SR_MODF)) ||
        ((sr & SPI_SR_FRLVL) != SPI_FRLVL_EMPTY) ||
        ((spi->CR1 & SPI_CR1_SPE) == 0U))
    {
        as5047p_spi_recover(dev);
    }

    as5047p_tCSH_delay();
    dev->cs_port->BSRR = ((uint32_t)dev->cs_pin) << 16U; /* CS low */

    t0 = DWT_GetTicks();
    while (((sr = spi->SR) & SPI_SR_TXE) == 0U) {
        if (sr & (SPI_SR_OVR | SPI_SR_FRE | SPI_SR_MODF)) {
            goto fail_error;
        }
        if (DWT_GetElapsedTicks(t0) > TO) {
            goto fail_timeout;
        }
    }

    *(__IO uint16_t *)&spi->DR = tx;

    t0 = DWT_GetTicks();
    while (((sr = spi->SR) & SPI_SR_RXNE) == 0U) {
        if (sr & (SPI_SR_OVR | SPI_SR_FRE | SPI_SR_MODF)) {
            goto fail_error;
        }
        if (DWT_GetElapsedTicks(t0) > TO) {
            goto fail_timeout;
        }
    }

    *rx = *(__IO uint16_t *)&spi->DR;

    t0 = DWT_GetTicks();
    while ((spi->SR & SPI_SR_BSY) != 0U) {
        if (DWT_GetElapsedTicks(t0) > TO) {
            goto fail_timeout;
        }
    }

    dev->cs_port->BSRR = dev->cs_pin; /* CS high */

    /* bit14 为 EF 错误标志，表示上一帧命令/通信存在错误 */
    dev->errorflag = (uint8_t)((*rx & 0x4000U) != 0U);

    return AS5047P_OK;

fail_error:
    dev->cs_port->BSRR = dev->cs_pin;
    as5047p_spi_recover(dev);
    return AS5047P_ERROR;

fail_timeout:
    dev->cs_port->BSRR = dev->cs_pin;
    as5047p_spi_recover(dev);
    return AS5047P_TIMEOUT;
}


/* AS5047P 使用 pipeline 读寄存器：
 * 第 1 帧发送读命令，第 2 帧发送 NOP 并取回上一帧请求的数据。
 */
AS5047P_OPTIMIZE
static AS5047P_Status_t as5047p_pipeline_read(AS5047P_Handle_t *dev,
                                              uint16_t cmd,
                                              uint16_t *frame)
{
    if (!dev || !frame) return AS5047P_ERROR;

    uint16_t dummy = 0U;

    AS5047P_Status_t st = as5047p_spi_transfer16(dev, cmd, &dummy);
    if (st != AS5047P_OK) return st;

    return as5047p_spi_transfer16(dev, AS5047P_CMD_READ_NOP, frame);
}


/* 角度读取高速路径：
 * 专门为高频控制循环优化，展开 pipeline 两帧读取流程，
 * 减少函数调用和通用逻辑开销。
 */
AS5047P_OPTIMIZE
static AS5047P_Status_t as5047p_pipeline_read_angle_fast(AS5047P_Handle_t *dev,
                                                         uint16_t *raw)
{
    SPI_TypeDef *spi = dev->hspi->Instance;
    const uint32_t TO = 3400U; /* 约 20us @170MHz */
    uint32_t t0;
    uint32_t sr;
    uint16_t frame;

    sr = spi->SR;
    if ((sr & (SPI_SR_OVR | SPI_SR_FRE | SPI_SR_MODF)) ||
        ((sr & SPI_SR_FRLVL) != SPI_FRLVL_EMPTY) ||
        ((spi->CR1 & SPI_CR1_SPE) == 0U)) {
        as5047p_spi_recover(dev);
    }

    /* 第一帧：发送 ANGLEUNC 读命令 */
    as5047p_tCSH_delay();
    dev->cs_port->BSRR = ((uint32_t)dev->cs_pin) << 16U;

    t0 = DWT_GetTicks();
    while (((sr = spi->SR) & SPI_SR_TXE) == 0U) {
        if (sr & (SPI_SR_OVR | SPI_SR_FRE | SPI_SR_MODF)) goto fail;
        if (DWT_GetElapsedTicks(t0) > TO) goto fail;
    }

    *(__IO uint16_t *)&spi->DR = AS5047P_CMD_READ_ANGLEUNC;

    t0 = DWT_GetTicks();
    while (((sr = spi->SR) & SPI_SR_RXNE) == 0U) {
        if (sr & (SPI_SR_OVR | SPI_SR_FRE | SPI_SR_MODF)) goto fail;
        if (DWT_GetElapsedTicks(t0) > TO) goto fail;
    }

    /* 第一帧返回的是上一帧数据，这里丢弃 */
    (void)*(__IO uint16_t *)&spi->DR;

    t0 = DWT_GetTicks();
    while ((spi->SR & SPI_SR_BSY) != 0U) {
        if (DWT_GetElapsedTicks(t0) > TO) goto fail;
    }

    dev->cs_port->BSRR = dev->cs_pin;

    /* 第二帧：发送 NOP，取回 ANGLEUNC 数据 */
    as5047p_tCSH_delay();
    dev->cs_port->BSRR = ((uint32_t)dev->cs_pin) << 16U;

    t0 = DWT_GetTicks();
    while (((sr = spi->SR) & SPI_SR_TXE) == 0U) {
        if (sr & (SPI_SR_OVR | SPI_SR_FRE | SPI_SR_MODF)) goto fail;
        if (DWT_GetElapsedTicks(t0) > TO) goto fail;
    }

    *(__IO uint16_t *)&spi->DR = AS5047P_CMD_READ_NOP;

    t0 = DWT_GetTicks();
    while (((sr = spi->SR) & SPI_SR_RXNE) == 0U) {
        if (sr & (SPI_SR_OVR | SPI_SR_FRE | SPI_SR_MODF)) goto fail;
        if (DWT_GetElapsedTicks(t0) > TO) goto fail;
    }

    frame = *(__IO uint16_t *)&spi->DR;

    t0 = DWT_GetTicks();
    while ((spi->SR & SPI_SR_BSY) != 0U) {
        if (DWT_GetElapsedTicks(t0) > TO) goto fail;
    }

    dev->cs_port->BSRR = dev->cs_pin;

    if (frame & 0x4000U) {
        dev->errorflag = 1U;
        return AS5047P_ERROR;
    }

    *raw = frame & 0x3FFFU;
    dev->errorflag = 0U;

    return AS5047P_OK;

fail:
    dev->cs_port->BSRR = dev->cs_pin;
    as5047p_spi_recover(dev);
    return AS5047P_TIMEOUT;
}


/* 读取 ERRFL 会清除 AS5047P 的粘滞错误位：
 * PARERR / INVCOMM / FRERR。
 */
AS5047P_OPTIMIZE
static AS5047P_Status_t as5047p_read_errfl_and_clear(AS5047P_Handle_t *dev,
                                                     uint16_t *errfl)
{
    if (!dev) return AS5047P_ERROR;

    uint16_t frame = 0U;

    AS5047P_Status_t st =
        as5047p_pipeline_read(dev, as5047p_make_read_cmd(AS5047P_REG_ERRFL), &frame);

    if (st != AS5047P_OK) return st;

    if (errfl) {
        *errfl = frame & 0x3FFFU;
    }

    return AS5047P_OK;
}


/* 通用 14bit 寄存器快速读取。
 * check_ef = 1 时检查 EF 位，适合 MAG / ANGLE 等有效数据寄存器。
 */
AS5047P_OPTIMIZE
static AS5047P_Status_t as5047p_read_register14_fast(AS5047P_Handle_t *dev,
                                                     uint16_t cmd,
                                                     uint16_t *data,
                                                     uint8_t check_ef)
{
    if (!dev || !data) return AS5047P_ERROR;

    uint16_t frame = 0U;

    AS5047P_Status_t st = as5047p_pipeline_read(dev, cmd, &frame);
    if (st != AS5047P_OK) return st;

    if (check_ef && (frame & 0x4000U)) {
        dev->errorflag = 1U;
        return AS5047P_ERROR;
    }

    *data = frame & 0x3FFFU;
    dev->errorflag = 0U;

    return AS5047P_OK;
}


/* 带重试和错误清除的寄存器读取。
 * 只在快速路径失败后使用，避免正常控制循环被复杂恢复逻辑拖慢。
 */
AS5047P_OPTIMIZE
static AS5047P_Status_t as5047p_read_register14_recover(AS5047P_Handle_t *dev,
                                                        uint16_t cmd,
                                                        uint16_t *data,
                                                        uint8_t check_ef)
{
    if (!dev || !data) return AS5047P_ERROR;

    AS5047P_Status_t st = AS5047P_ERROR;
    uint16_t frame = 0U;
    uint16_t errfl = 0U;

    for (uint8_t i = 0U; i < AS5047P_READ_RETRY_MAX; i++) {
        st = as5047p_pipeline_read(dev, cmd, &frame);
        if (st != AS5047P_OK) {
            continue;
        }

#if AS5047P_ENABLE_FRAME_PARITY_CHECK
        if (as5047p_calc_parity(frame) != ((frame >> 15U) & 0x1U)) {
            dev->errorflag = 1U;
            (void)as5047p_read_errfl_and_clear(dev, &errfl);
            st = AS5047P_ERROR;
            continue;
        }
#endif

        if (check_ef && (frame & 0x4000U)) {
            dev->errorflag = 1U;
            (void)as5047p_read_errfl_and_clear(dev, &errfl);
            st = AS5047P_ERROR;
            continue;
        }

        *data = frame & 0x3FFFU;
        dev->errorflag = 0U;

        return AS5047P_OK;
    }

    return st;
}


/* 读取原始角度。
 * 优先走高速路径；失败后清 ERRFL，再进入带重试的恢复路径。
 */
AS5047P_Status_t AS5047P_ReadRawAngle(AS5047P_Handle_t *dev, uint16_t *raw)
{
    if (!dev || !raw) return AS5047P_ERROR;

    AS5047P_Status_t st = as5047p_pipeline_read_angle_fast(dev, raw);

    if (st != AS5047P_OK) {
        uint16_t errfl = 0U;

        (void)as5047p_read_errfl_and_clear(dev, &errfl);

        st = as5047p_read_register14_recover(dev,
                                             AS5047P_CMD_READ_ANGLEUNC,
                                             raw,
                                             1U);
        if (st != AS5047P_OK) {
            return st;
        }
    }

    dev->raw_angle = *raw;
    dev->angle_rad = (*raw) * AS5047P_ANGLE_RAD_SCALE;

    return AS5047P_OK;
}


/* 读取磁场幅值 MAG，可用于检查磁铁安装距离和磁场强度是否合理 */
AS5047P_Status_t AS5047P_ReadMagnitude(AS5047P_Handle_t *dev, uint16_t *mag)
{
    if (!dev || !mag) return AS5047P_ERROR;

    AS5047P_Status_t st =
        as5047p_read_register14_fast(dev,
                                     as5047p_make_read_cmd(AS5047P_REG_MAG),
                                     mag,
                                     1U);

    if (st == AS5047P_OK) return st;

    uint16_t errfl = 0U;
    (void)as5047p_read_errfl_and_clear(dev, &errfl);

    return as5047p_read_register14_recover(dev,
                                           as5047p_make_read_cmd(AS5047P_REG_MAG),
                                           mag,
                                           1U);
}


/* 读取并清除 ERRFL */
AS5047P_Status_t AS5047P_ReadErrfl(AS5047P_Handle_t *dev, uint16_t *errfl)
{
    return as5047p_read_errfl_and_clear(dev, errfl);
}


/* 只清除错误标志，不关心返回的 ERRFL 内容 */
AS5047P_Status_t AS5047P_ClearErrorFlag(AS5047P_Handle_t *dev)
{
    return as5047p_read_errfl_and_clear(dev, NULL);
}


/* AS5047P 底层驱动初始化：
 * 只负责 SPI/CS 绑定、DWT 计数器启用、启动错误清除和首次角度读取测试。
 * 不在这里处理跨圈、速度估计、电角度等 Motor/Sensor 层逻辑。
 */
uint8_t AS5047P_RW_Init(AS5047P_Handle_t *dev,
                        SPI_HandleTypeDef *hspi,
                        GPIO_TypeDef *cs_port,
                        uint16_t cs_pin)
{
    if (!dev || !hspi || !cs_port) {
        return 0U;
    }

    dev->hspi = hspi;
    dev->cs_port = cs_port;
    dev->cs_pin = cs_pin;

    as5047p_enable_dwt_cycle_counter();

    dev->initialized = 0U;
    dev->errorflag = 0U;
    dev->raw_angle = 0U;
    dev->angle_rad = 0.0f;

    /* 初始化时保持 CS 高电平，避免误触发 SPI transaction */
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);

    /* 上电后等待角度数据有效 */
    HAL_Delay(10U);

    /* 清掉上电阶段可能残留的粘滞错误 */
    uint16_t errfl = 0U;
    (void)as5047p_read_errfl_and_clear(dev, &errfl);
    (void)as5047p_read_errfl_and_clear(dev, &errfl);

    /* 首次读取角度作为链路自检 */
    uint16_t raw = 0U;
    AS5047P_Status_t st = AS5047P_ReadRawAngle(dev, &raw);
    if (st != AS5047P_OK) {
        return 0U;
    }

    dev->initialized = 1U;

    return 1U;
}


