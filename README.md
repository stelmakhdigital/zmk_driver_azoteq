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

Added on a device with a touchpad...

```
# Enable TPS43 input driver (pointing device)
CONFIG_INPUT_TPS43=y
CONFIG_INPUT_TPS43_SENSITIVITY=100
```

```
&i2c0 {
    status = "okay";
    clock-frequency = <I2C_BITRATE_FAST>;
    pinctrl-0 = <&i2c0_azoteq_default>;
    pinctrl-1 = <&i2c0_azoteq_sleep>;
    pinctrl-names = "default", "sleep";

    tps43_trackpad: trackpad@74 {
        compatible = "azoteq,tps43";
        reg = <0x74>;
        status = "okay";
        
        /* Device variant and resolution */
        variant = "43mm";                    /* 43mm x 40mm trackpad */
        resolution-x = <2048>;               /* X-axis resolution */
        resolution-y = <1792>;               /* Y-axis resolution */
        
        /* GPIO connections (adjust pin numbers for your board) */
        int-gpios = <&pro_micro 21 GPIO_ACTIVE_HIGH>;  /* interrupt pin */
        rst-gpios = <&pro_micro 20 GPIO_ACTIVE_HIGH>;  /* Reset pin */
        
        /* Touch detection settings */
        touch-threshold = <40>;              /* Touch sensitivity (lower = more sensitive) */
        max-touches = <5>;                   /* Maximum simultaneous touches */
        palm-reject-threshold = <100>;       /* Ignore touches larger than this */
        
        /* Feature enablement */
        gesture-enable;                      /* Enable gesture recognition */
        
        /* Communication settings */
        i2c-timeout-ms = <10>;               /* I2C timeout in milliseconds */
        
        /* Input driver settings */
        sensitivity = <100>;                  /* Sensitivity multiplier (percentage) */
    };
};

```


Only if there is one device (Central)

```
/ {
    tps43_input: tps43_input {
        compatible = "zmk,input-listener";
        device = <&tps43>;
    };
};
```

Only if there is SPLIT device

> Central
```
/ {
    split_inputs {
        #address-cells = <1>;
        #size-cells = <0>;

        tps43_split: tps43_split@0 {
            compatible = "zmk,input-split";
            reg = <0>;
            /* No device property here - this is a proxy on central side */
        };
    };

    tps43_listener: tps43_listener {
        compatible = "zmk,input-listener";
        device = <&tps43_split>;
        status = "okay";
    };
};
```

> Peripheral
```
/ {
    split_inputs {
        #address-cells = <1>;
        #size-cells = <0>;

        tps43_split: tps43_split@0 {
            compatible = "zmk,input-split";
            reg = <0>;
            device = <&tps43_trackpad>;
        };
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
