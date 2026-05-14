# ch9431 SPI CAN Driver
---------------------------------------
This driver can only work with SPI CAN function in these WCH devices: **CH9431**.

---

## Integrated into your system method 1
---------------------------------------
> **Note:** If you are using **dts device tree** to set up spi and driver, you can read this method, otherwise please refer to method 2.

1. Please copy the driver file to the package directory which be used to add additional drivers.
2. Please add the relevant Makefile and Kconfig like other drivers, generally you can copy one from other driver then modify it.
3. Run the `make menuconfig` and select the ch9431 can support at "modules" item.
4. Define the spi structure on your dts file similar the follow:

```dts
spidev@1 {
    #address-cells = <1>;
    #size-cells = <1>;
    compatible = "wch,ch9431";
    reg = <0x0>;
    spi-max-frequency = <12000000>;
    interrupt-gpio = <&gpio0 12 IRQ_TYPE_LEVEL_LOW>;
}
Notice that the irq request method cannot be supported in this way in some platforms. You can contact us for other methods.

## Integrated into your system method 2
---------------------------------------
1. Please copy the driver file to the kernel directory: $kernel_src/drivers/net/can/spi/

2. Please add the followed txt into the kernel file: $kernel_src/drivers/net/can/spi/Kconfig
config CAN_CH9431
    tristate "WCH CH9431 SPI CAN controllers"
    depends on CAN_DEV && SPI && HAS_DMA
    ---help---
      Driver for the WCH CH9431 SPI CAN
      controllers.

3. Add the follow define into the $kernel_src/drivers/net/can/spi/Makefile for compile the driver.
obj- $ (CONFIG_CAN_CH9431) += ch9431.o

4.Run the make menuconfig and select the ch9431 CAN support at the:
Networking support -> CAN bus subsystem support -> CAN Device Drivers -> CAN SPI interfaces -> WCH CH9431 SPI CAN controllers and save the config.

5. Define the spi0_board_info object on your board file similar the follow:
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
---------------------------------------
## Contact
Any question, you can send feedback to mail: mailto:tech@wch.cn
---------------------------------------
