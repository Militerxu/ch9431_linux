# ch9431 SPI CAN 驱动

该驱动仅适用于以下 WCH 设备中的 SPI CAN 功能：

- CH9431

## 集成到系统的方法 1

如果你使用 dts 设备树来配置 SPI 和驱动，可以参考此方法；否则请参考方法 2。

1. 请将驱动文件复制到用于添加额外驱动的软件包目录中。

2. 请像其他驱动一样添加相应的 `Makefile` 和 `Kconfig`。通常可以从其他驱动复制一份，然后再进行修改。

3. 运行 `make menuconfig`，并在 `"modules"` 项中选择 `ch9431 can support`。

4. 在你的 dts 文件中定义类似如下的 SPI 结构：

```dts
spidev@1 {
	#address-cells = <1>;
	#size-cells = <1>;
	compatible = "wch,ch9431";
	reg = <0x0>;
	spi-max-frequency = <12000000>;
	interrupt-gpio = <&gpio0 12 IRQ_TYPE_LEVEL_LOW>;
}
```

请注意，在某些平台上，这种方式不支持 irq 请求方法。如需其他方法，可以联系我们。

## 集成到系统的方法 2

1. 请将驱动文件复制到内核目录：`$kernel_src/drivers/net/can/spi/`

2. 请将以下文本添加到内核文件：`$kernel_src/drivers/net/can/spi/Kconfig`

```text
config CAN_CH9431
	tristate "WCH CH9431 SPI CAN controllers"
	depends on CAN_DEV && SPI && HAS_DMA
	---help---
	  Driver for the WCH CH9431 SPI CAN
	  controllers.
```

3. 将以下定义添加到 `$kernel_src/drivers/net/can/spi/Makefile`，用于编译该驱动。

```makefile
obj-$(CONFIG_SERIAL_CH9431) += ch9431.o
```

4. 运行 `make menuconfig`，并在以下位置选择 `ch9431 CAN support`，然后保存配置。

```text
Networking support-> CAN bus subsystem support -> CAN Device Drivers -> CAN SPI interfaces ->  WCH CH9431 SPI CAN controllers
```

5. 在你的板级文件中定义类似如下的 `spi0_board_info` 对象：

```c
static struct spi_board_info spi0_board_info[] __initdata = {
	{
		.modalias = "wch,ch9431",
		.platform_data = NULL,
		.max_speed_hz = 100 * 1000,
		.bus_num = 0,
		.chip_select = 0,
		.mode = SPI_MODE_0,
		.controller_data = &spi0_csi[0],
		.irq = IRQ_EINT(25),
	}
};
```

## 注意

如有任何问题，可以发送反馈邮件至：tech@wch.cn
