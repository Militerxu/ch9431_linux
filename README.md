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
