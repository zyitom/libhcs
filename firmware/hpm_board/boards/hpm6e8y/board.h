#pragma once

#include <stdbool.h>
#include <stdint.h>

#include <hpm_common.h>
#include <hpm_soc.h>

/*
 * HCS EtherCAT 桥板 "hpm6e8y" -- HPM6E00 芯片上运行 HPM6E80 固件, 硬件等同
 * hpm6e00evk。与 hpm6e80ivm1 的差异在 XPI NOR 的 FCFG以及 EtherCAT PHY 走线:
 * 本板使用 HPM6E*Y* 的片内 100M PHY。
 *
 * 遵循共享 bootloader 所用的仓库板级 API(board_init、board_init_usb(void)、
 * 强制驻留钩子、flash/XPI 宏), 并补充桥固件所需的双核钩子 board_init_core1
 * 与 EtherCAT 钩子 board_init_pmp、board_init_ethercat。
 */

#define BOARD_NAME          "HCS_ECAT_Bridge_HPM6E8Y"
#define BOARD_UF2_SIGNATURE (0x0A4D5048UL)

/* 第二核镜像的加载/入口地址(SDK 板级约定, 由 core0 侧 ecat_main.c 的
 * multicore_release_cpu 消费)。 */
#define SEC_CORE_IMG_START CORE1_ILM_LOCAL_BASE

#define BOARD_FLASH_BASE_ADDRESS (0x80000000UL)
#define BOARD_FLASH_SIZE         (4 * SIZE_1MB)
/* app 镜像必须结束于 2 MiB 偏移处的 EtherCAT flash 模拟 EEPROM 区
 * (BOARD_ECAT_FLASH_EMULATE_EEPROM_ADDR)之下; bootloader 以此限制可接受的
 * 镜像大小。 */
#define BOARD_APP_FLASH_END_OFFSET (0x200000UL)

/* XPI NOR 配置选项(与 hpm6e00evk 同一颗 flash), 供 bootloader 的 flash 写入器
 * 与 EtherCAT EEPROM 模拟 port 层使用。 */
#define BOARD_APP_XPI_NOR_XPI_BASE     (HPM_XPI0)
#define BOARD_APP_XPI_NOR_CFG_OPT_HDR  (0xfcf90002U)
#define BOARD_APP_XPI_NOR_CFG_OPT_OPT0 (0x00000007U)
#define BOARD_APP_XPI_NOR_CFG_OPT_OPT1 (0x00001000U)
#define BOARD_BGPR                     HPM_BGPR0

/* 供 SDK port 层(samples/ethercat/port)消费的 EtherCAT 定义。 */
#define BOARD_ECAT_SUPPORT_PORT1 (1)
#define BOARD_ECAT_SUPPORT_PORT2 (0)

/* RUN/ERROR 状态 LED: GPIO LED 扫描判定 PC20(绿)/PC21(红)为 "EtherCAT 中部"
 * 指示灯, 两者均带 ESC0_CTR 复用功能(PC20 = CTR_2, PC21 = CTR_3), ESC 由 AL
 * 状态机驱动它们。board.c 的 init_esc_pins 负责焊盘复用, 下面的下标选择对应
 * 的 CTR。 */
#define BOARD_ECAT_SUPPORT_RUN_ERROR_LED (1)

/* 本硬件上 ESC 端口链路信号低有效。 */
#define BOARD_ECAT_PORT0_LINK_INVERT true
#define BOARD_ECAT_PORT1_LINK_INVERT true
#define BOARD_ECAT_PORT2_LINK_INVERT false

/* HPM6E*Y* 片内 PHY 的复位输入是内部 PV/PW GPIO 焊盘。 */
#define BOARD_ECAT_PHY0_RESET_GPIO            HPM_GPIO0
#define BOARD_ECAT_PHY0_RESET_GPIO_PORT_INDEX GPIO_DO_GPIOV
#define BOARD_ECAT_PHY0_RESET_PIN_INDEX       (12)
#define BOARD_ECAT_PHY1_RESET_GPIO            HPM_GPIO0
#define BOARD_ECAT_PHY1_RESET_GPIO_PORT_INDEX GPIO_DO_GPIOW
#define BOARD_ECAT_PHY1_RESET_PIN_INDEX       (12)
#define BOARD_ECAT_PHY_RESET_LEVEL            (0)

/* 必须与 board.c 配置的 ESC0_CTR_y 功能一致。PA25(CTR_0)与 PA28(CTR_1)是
 * NMII_LINK 源; PC20(CTR_2)与 PC21(CTR_3)是 RUN/ERROR 状态 LED。 */
#define BOARD_ECAT_NMII_LINK0_CTRL_INDEX 1
#define BOARD_ECAT_NMII_LINK1_CTRL_INDEX 0
#define BOARD_ECAT_LED_RUN_CTRL_INDEX    2
#define BOARD_ECAT_LED_ERROR_CTRL_INDEX  3

#define BOARD_ECAT_PHY_ADDR_OFFSET (0U)
#define BOARD_ECAT_PORT0_PHY_ADDR  (2U)
#define BOARD_ECAT_PORT1_PHY_ADDR  (1U)
/* 板上丝印仍是 EtherCAT0 = IN、EtherCAT1 = OUT。内部要让 ESC 枚举成功, 必须
 * 把这些物理链路报告给相反的逻辑端口: 物理 EtherCAT0/IN 供给 ESC 端口 1,
 * 物理 EtherCAT1/OUT 供给 ESC 端口 0。除非板上走线或底层 ESC 端口分配改变,
 * 不要把它设为 0 -- swap=0 实测过, 主站报 "No EtherCAT slave found"。 */
#define BOARD_ECAT_SWAP_PHY_LINK_TO_ESC_PORT (1)

/* 仅在使用真实 I2C EEPROM 时被 port 层引用; 本项目用 flash 模拟, 但宏必须
 * 存在。 */
#define BOARD_ECAT_INIT_EEPROM_I2C     HPM_I2C1
#define BOARD_ECAT_INIT_EEPROM_I2C_CLK clock_i2c1

/* 模拟 ESC EEPROM 的 flash 偏移(见 BOARD_APP_FLASH_END_OFFSET)。 */
#define BOARD_ECAT_FLASH_EMULATE_EEPROM_ADDR (0x200000)
#define BOARD_ECAT_FLASH_EMULATE_EEPROM_SIZE (0x10000)

/* FoE 暂存区。
 *
 * 经 FoE 收到(或经 USB 由自检路径推入)的固件镜像, 由运行中的 app 经跨核
 * flash RPC 写到这里, 冷复位后由 bootloader 装入 app 槽。app 自己永远不能写
 * app 槽: core0 正从它 XIP 执行, 擦除它等于把正在执行擦除的代码脚下的地面
 * 抽走。
 *
 * 它位于模拟 EEPROM 之上、flash 上半部其他东西都不用的区域。这个位置是刻意
 * 选的: 不必移动任何现有边界 -- BOARD_APP_FLASH_END_OFFSET 不动、app 链接
 * 脚本不动、bootloader 接受 DFU 下载的范围也不动。
 *
 * 首个扇区存放暂存元数据(与 app 元数据扇区相同的 append-only DataSlot 格式,
 * commit-barrier 语义因此共享而非另写一套); 候选镜像紧随其后。
 */
#define BOARD_FOE_STAGING_ADDR                                                                     \
    (BOARD_ECAT_FLASH_EMULATE_EEPROM_ADDR + BOARD_ECAT_FLASH_EMULATE_EEPROM_SIZE)
#define BOARD_FOE_STAGING_METADATA_SIZE (0x1000UL)
#define BOARD_FOE_STAGING_END_OFFSET    (BOARD_FLASH_SIZE)

#ifdef __cplusplus
extern "C" {
#endif

void board_init(void);
void board_init_core1(void);
void board_init_console(void);
void board_init_pmp(void);
void board_init_usb(void);
void board_init_ethercat(ESC_Type* ptr);

/* 仅当 UART TX FIFO 有空位时写一个控制台字节, 没有空位则返回 false。SDK 的
 * console_send_byte() 会在 THR 空时忙等(115200 波特下每字节约 87 us), 对同时
 * 要服务 USB 与 CAN 的核不可接受 -- 排空一段日志突发会把数据通路卡住毫秒级。
 * 供跨核诊断排空使用(core1 不能 printf; 见 ecat/common/xcore_diag.hpp)。 */
bool board_console_try_send_byte(uint8_t byte);

/* 由软件驱动两个片内 PHY 的 ESC NMII_LINK(片内 PHY 不向 ESC CTR 输入馈
 * LINK 引脚, SDK 默认值下 ESC 看不到任何链路)。须在 ecat_hardware_init 之后
 * 调用, 定义见 board.c。 */
void board_ecat_set_internal_phy_link(bool port0_up, bool port1_up);

typedef struct board_ecat_phy_status {
    bool port0_read_ok;
    bool port0_link_up;
    uint16_t port0_bmcr;
    uint16_t port0_bmsr;
    uint16_t port0_id1;
    uint16_t port0_id2;
    uint16_t port0_rmsr_p7;
    bool port1_read_ok;
    bool port1_link_up;
    uint16_t port1_bmcr;
    uint16_t port1_bmsr;
    uint16_t port1_id1;
    uint16_t port1_id2;
    uint16_t port1_rmsr_p7;
} board_ecat_phy_status_t;

/* 经已验证可靠的 ENET0 SMI 通路(PA30/PA31)读取两个内部 JL1111 PHY, 然后把
 * PA30/PA31 还原给 ESC0_MDIO/MDC。 */
bool board_ecat_get_internal_phy_status(board_ecat_phy_status_t* status);

/* 经已验证可靠的 ENET0 SMI 通路把两个内部 JL1111 PHY 强制为 MII 模式。须在
 * EtherCAT port 层释放 PHY 复位之后调用。 */
bool board_ecat_configure_internal_phy_mii_mode(void);

/* 用真实 PHY 链路状态刷新 ESC NMII_LINK。两个内部 PHY 都读不到时返回 false,
 * 此时保留先前的 ESC 链路状态不动。 */
bool board_ecat_refresh_internal_phy_link(void);
bool board_ecat_wait_internal_phy_link(uint32_t timeout_ms);

/* Bootloader 专用助手: 让 PB24(低有效, 内部上拉)贯穿复位保持有效, 强制
 * bootloader 驻留 DFU 模式。本板没有任何按键 -- PB24 是裸焊盘, 即把它短接到
 * GND。见 board.c。 */
bool board_check_bootloader_force_stay_requested(void);

void board_delay_us(uint32_t us);
void board_delay_ms(uint32_t ms);

#ifdef __cplusplus
}
#endif
