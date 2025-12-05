# Драйвер ZMK для тачпадов Azoteq IQS5XX

## Совместимость

Этот драйвер должен работать с любым тачпадом на базе IQS5XX (TPS43 или TPS65).

## Поддержка

- Движение тачпада.
- Одиночное касание: регистрируется как левый клик.
- Касание двумя пальцами: регистрируется как правый клик.
- Нажатие и удержание: регистрируется как непрерывный левый клик (перетаскивание).
- Вертикальная прокрутка.
- Горизонтальная прокрутка.

## Использование

- Укажите узел с совместимостью "azoteq,tps43" внутри узла i2c в оверлее клавиатуры.
- Ссылайтесь на него из слушателя ввода:


```
CONFIG_INPUT_TPS43=y
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
        
        /* GPIO connections */
        rdy-gpios = <&pro_micro 21 GPIO_ACTIVE_HIGH>;  /* Ready pin */
        rst-gpios = <&pro_micro 20 GPIO_ACTIVE_HIGH>;  /* Reset pin */

        suspend-timeout-ms = <60000>;  /* 60 seconds */
        enable-power-management;
        
        sensitivity = <110>;           /* 100% = normal */
        scroll-sensitivity = <15>;     /* 50% = normal */

        scroll;
        two-finger-tap;
        single-tap;
        press-and-hold;

        switch-xy;
        invert-scroll-y;
    };
};
```


Только если одно устройство (Центральное)

```
/ {
    tps43_input: tps43_input {
        compatible = "zmk,input-listener";
        device = <&tps43_trackpad>;
    };
};
```

Только если SPLIT устройство

> Центральное
```
/ {
    split_inputs {
        #address-cells = <1>;
        #size-cells = <0>;

        tps43_split: tps43_split@0 {
            compatible = "zmk,input-split";
            reg = <0>;
            /* Здесь нет свойства device - это прокси на центральной стороне */
        };
    };

    tps43_listener: tps43_listener {
        compatible = "zmk,input-listener";
        device = <&tps43_split>;
        status = "okay";
    };
};
```

> Периферийное
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


> Для настройки тачпада Azoteq требуется 5 пинов!

Питание:
3V на nice!nano -> VDD на IQS5xx.
G (Земля) на nice!nano -> GND на IQS5xx.


Сигналы I2C:
SDA на nice!nano -> SDA на IQS5xx.
SCL на nice!nano -> SCL на IQS5xx.

Пин готовности данных / прерывания:

Пин "DR" или "RDY" на IQS5xx -> Любой доступный GPIO на nice!nano. В devicetree этот пин указывается как rdy-gpios.

Пин "RST" используется для инициализации сбоса устройства.


## Правильная последовательность работы драйвера

```txt
1. Включение питания / Аппаратный сброс
   └─> Ожидание 10мс
   └─> RST: LOW (10мс) → HIGH
   └─> Ожидание ~600мс для загрузки прошивки

2. Проверка флага SHOW_RESET (0x000F бит 0)
   └─> Опрос до появления флага

3. Подтверждение сброса
   └─> Запись ACK_RESET (0x0431 = 0x80)

4. Конфигурация устройства
   └─> System Config 1 (0x058F) - режимы событий
   └─> XY Config (0x0669) - настройки осей
   └─> Настройки фильтров, жестов и т.д.

5. Завершение настройки
   └─> Запись SETUP_COMPLETE (0x058E = 0x40)

6. Конфигурация прерывания GPIO (RDY)
   └─> ПОСЛЕ полной конфигурации

```