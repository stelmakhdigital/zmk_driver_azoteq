# ZMK driver for Azoteq IQS5XX trackpads

## Compatibility

This driver should work with any IQS5XX based trackpad for TPS43 model.

## Support

- Trackpad movement.
- Single finger tap: Reported as a left click.
- Two finger tap: Reported as a right click.
- Press and hold: Reported as a continuos left click (allows click and drag).
- Vertical scroll.
- Horizontal scroll.

## Usage

- Specify a node with the "azoteq,iqs5xx" compatible inside an i2c node in your keyboard overlay.
- Reference it from an input listener:

```
/ {
    tps43_input: tps43_input {
        compatible = "zmk,input-listener";
        device = <&tps43>;
    };
};

&i2c1 {
    status = "okay";
    tps43: iqs5xx@74 {
        status = "okay";
        compatible = "azoteq,iqs5xx";
        reg = <0x74>;

        reset-gpios = <&pro_micro 14 GPIO_ACTIVE_LOW>;
        rdy-gpios = <&pro_micro 15 GPIO_ACTIVE_HIGH>;

        /*
         * Potentially non-exhaustive list of configuration options.
         * See: dts/bindings/input/azoteq,iqs5xx-common.yaml for a full list.
         */
        one-finger-tap = <true>;
        press-and-hold = <true>;
        press-and-hold-time = <250>;
        two-finger-tap = <true>;

        scroll = <true>;
        natural-scroll-y = <true>;
        natural-scroll-x = <true>;

        bottom-beta = <5>;
        stationary-threshold = <5>;

        switch-xy = <false>;
    };
};
```

> 5 pins are needed to configure the azoteq trackpad!

Power:
3V on the nice!nano -> VDD on the IQS5xx.
G (Ground) on the nice!nano -> GND on the IQS5xx.


I2C Signals:
SDA (Feather pin labeled "SDA") on the nice!nano -> SDA on the IQS5xx.
SCL (Feather pin labeled "SCL") on the nice!nano -> SCL on the IQS5xx.
Data Ready / Interrupt Pin:

The IQS5xx "DR" or "RDY" pin -> Any available GPIO on the nice!nano.
For example, you can use D2, D3, or A0—whichever is free in your design. In devicetree, you’ll reference this pin under dr-gpios.