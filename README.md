# LIS3DH

[![Hex.pm](https://img.shields.io/hexpm/v/lis3dh.svg)](https://hex.pm/packages/lis3dh)
[![License: Apache-2.0](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

Elixir driver for the
[STMicroelectronics LIS3DH](https://www.st.com/en/mems-and-sensors/lis3dh.html)
3-axis MEMS accelerometer, connected over I²C.

Built on [Wafer](https://harton.dev/james/wafer), so it's independent of any
particular I²C backend — use [`circuits_i2c`](https://hex.pm/packages/circuits_i2c)
on a Nerves target, [`circuits_ft232h`](https://hex.pm/packages/circuits_ft232h)
when developing on a laptop, or any other `Wafer.I2C` implementation.

I'm prototyping against [Adafruit's breakout](https://www.adafruit.com/product/2809).

## Features

- Configurable operating mode (low-power 8-bit, normal 10-bit, high-resolution
  12-bit), output data rate, ±2/4/8/16 g range, per-axis enables, and
  block-data-update.
- Acceleration reads scaled to m/s² for the active mode and range.
- High-pass filter configuration and `REFERENCE` register access.
- FIFO sampler (`LIS3DH.Sampler`) with bypass, stream, stream-to-FIFO, and
  FIFO modes.
- Inertial interrupts: free-fall, motion / wake-up, 4D and 6D orientation,
  and activity (sleep-to-wake).
- Single- and double-click / tap detection.
- Self-test toggle (per ST application note AN3308).
- Auxiliary ADC (3 channels) and the embedded temperature sensor.
- Routable interrupt pins (`INT1`, `INT2`) with polarity and latching
  controls.

## Usage

```elixir
iex> {:ok, conn} = Wafer.Driver.Circuits.I2C.acquire(bus_name: "i2c-1", address: 0x18)
iex> {:ok, acc}  = LIS3DH.acquire(conn: conn)
iex> {:ok, acc}  = LIS3DH.configure_accelerometer(acc, mode: :normal, odr: 100, range: 2)
iex> LIS3DH.read_accelerometer(acc)
{:ok, %{x: 0.157, y: -0.118, z: 9.083}}
```

`acquire/1` verifies the device's `WHO_AM_I` by default; pass
`verify_who_am_i: false` to skip it, or `reboot: true` to refresh the trim
registers from non-volatile memory before reading.

The 7-bit I²C address is `0x18` when the `SA0` pin is tied to GND, or `0x19`
when tied to VDD.

See the moduledoc and `LIS3DH.Sampler`, `LIS3DH.Interrupts`, and `LIS3DH.Click`
for the full API.

## Installation

The `lis3dh` package is [available on Hex](https://hex.pm/packages/lis3dh) and
can be installed by adding it to your dependencies in `mix.exs`:

```elixir
def deps do
  [
    {:lis3dh, "~> 0.1.0"}
  ]
end
```

Documentation for the latest release is on
[HexDocs](https://hexdocs.pm/lis3dh).

## GitHub mirror

This repository is mirrored [on GitHub](https://github.com/jimsynz/lis3dh)
from its primary location [on my Forgejo instance](https://harton.dev/james/lis3dh).
Feel free to raise issues and open PRs on either.

## License

This software is licensed under the terms of the
[Apache 2.0 license](https://www.apache.org/licenses/LICENSE-2.0).
