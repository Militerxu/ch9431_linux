<<<<<<< HEAD
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
=======
# CH9431 SPI CAN Driver

---------------------------------------

This driver can only work with the SPI CAN function in the WCH **CH9431** device.

## Integrated into your system method 1

---------------------------------------

> **Note:**  
> If you are using the DTS device tree to set up SPI and the driver, please refer to this method.  
> Otherwise, please refer to method 2.

1. Please copy the driver file to the package directory used to add additional drivers.

2. Please add the relevant `Makefile` and `Kconfig` entries like other drivers. Generally, you can copy them from another driver and modify them.

3. Run `make menuconfig` and select CH9431 CAN support as a module or built-in driver.

4. Define the SPI device node in your DTS file, similar to the following:

```dts
&spi0 {
    status = "okay";

    #address-cells = <1>;
    #size-cells = <0>;

    ch9431: can@0 {
        compatible = "wch,ch9431";
        reg = <0>;
        spi-max-frequency = <12000000>;
        interrupt-gpio = <&gpio0 12 IRQ_TYPE_LEVEL_LOW>;
    };
};
```

Notice that the IRQ request method may not be supported on some platforms.  
If the interrupt cannot be requested successfully, please adjust the GPIO and IRQ configuration according to your platform.

## Integrated into your system method 2

---------------------------------------

1. Please copy the driver file to the kernel directory:

```bash
$kernel_src/drivers/net/can/spi/
```

2. Please add the following text into the kernel file:

```bash
$kernel_src/drivers/net/can/spi/Kconfig
```

```kconfig
config CAN_CH9431
    tristate "WCH CH9431 SPI CAN controllers"
    depends on CAN_DEV && SPI && HAS_DMA
    help
      Driver for the WCH CH9431 SPI CAN controllers.
```

3. Add the following define into the kernel file:

```bash
$kernel_src/drivers/net/can/spi/Makefile
```

```makefile
obj-$(CONFIG_CAN_CH9431) += ch9431.o
```

4. Run `make menuconfig` and select CH9431 CAN support at:

```text
Networking support
  -> CAN bus subsystem support
    -> CAN Device Drivers
      -> CAN SPI interfaces
        -> WCH CH9431 SPI CAN controllers
```

Then save the configuration.

5. Define the `spi_board_info` object in your board file, similar to the following:

```c
static struct spi_board_info spi0_board_info[] __initdata = {
    {
        .modalias = "ch9431",
        .platform_data = NULL,
        .max_speed_hz = 12 * 1000 * 1000,
        .bus_num = 0,
        .chip_select = 0,
        .mode = SPI_MODE_0,
        .controller_data = &spi0_csi[0],
        .irq = IRQ_EINT(25),
    }
};
```

Notice that `.modalias` should match the `spi_device_id` table in the driver.

---------------------------------------

## Contact

Any question, you can send feedback to:

```text
tech@wch.cn
```

---------------------------------------
>>>>>>> 67d682868b008a8c8d50143106a6940a71e2864c
