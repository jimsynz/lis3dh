defmodule LIS3DH do
  @moduledoc """
  Driver for the STMicroelectronics LIS3DH 3-axis MEMS accelerometer.

  Communicates over I²C (or any other [Wafer](https://hex.pm/packages/wafer)
  transport that implements the `Wafer.I2C` protocol).

  ## Protocol notes

  The LIS3DH uses a standard byte-oriented I²C register protocol. Multi-byte
  reads and writes only auto-increment the register address when bit 7 of the
  sub-address is set; this driver sets that bit unconditionally on every
  transaction, which is harmless for single-byte access and required for
  bursts.

  ## I²C address

  The 7-bit address is `0b0011000x` where `x` is the value of the `SA0` pin
  (also called `SDO`):

    * `SA0 = GND` → `0x18` (default).
    * `SA0 = VDD` → `0x19`.

  ## Example

      {:ok, i2c} = Wafer.Driver.Circuits.I2C.acquire(bus_name: "i2c-1", address: 0x18)
      {:ok, acc} = LIS3DH.acquire(conn: i2c)
  """

  import Bitwise

  alias LIS3DH.Click
  alias LIS3DH.Config
  alias LIS3DH.Interrupts
  alias LIS3DH.Registers
  alias Wafer.Chip
  alias Wafer.Conn

  defstruct conn: nil, operating_mode: nil, range: nil

  @type t :: %__MODULE__{
          conn: Conn.t(),
          operating_mode: Config.operating_mode() | nil,
          range: Config.range() | nil
        }
  @type who_am_i :: byte
  @type axes :: %{x: float, y: float, z: float}
  @type acquire_option ::
          {:conn, Conn.t()}
          | {:verify_who_am_i, boolean}
          | {:reboot, boolean}

  @behaviour Wafer.Conn

  @default_i2c_address 0x18
  @expected_who_am_i 0x33
  @boot_delay_ms 5
  @ctrl_reg_5_boot_bit 7
  @gravity_ms2 9.80665
  @out_x_l 0x28
  @out_adc1_l 0x08
  @aux_adc_center_mv 1200
  @aux_adc_span_mv 400
  @temp_en_bit 6
  @adc_en_bit 7

  @doc """
  The default 7-bit I²C address (`0x18`, SA0 pin tied to GND). The alternate
  address `0x19` is selected by tying SA0 to VDD.
  """
  @spec default_i2c_address() :: 0x18
  def default_i2c_address, do: @default_i2c_address

  @doc """
  The expected `WHO_AM_I` value (`0x33`) returned by an unmodified LIS3DH.
  """
  @spec expected_who_am_i() :: 0x33
  def expected_who_am_i, do: @expected_who_am_i

  @doc """
  Wrap an existing Wafer connection in a `LIS3DH` struct.

  ## Options

    * `:conn` (required) — a Wafer connection that implements the `Wafer.I2C`
      protocol, e.g. `Wafer.Driver.Circuits.I2C` or `Wafer.Driver.Fake`.
    * `:verify_who_am_i` (default `true`) — when `true`, read `WHO_AM_I` and
      return `{:error, {:who_am_i_mismatch, got: byte, expected: 0x33}}` if
      the device does not identify as a LIS3DH.
    * `:reboot` (default `false`) — when `true`, set `CTRL_REG5.BOOT` to
      refresh the internal trim registers from non-volatile memory and block
      for #{@boot_delay_ms} ms before returning. Useful after power glitches
      or when you suspect the trim values have been corrupted.
  """
  @impl Wafer.Conn
  @spec acquire([acquire_option]) :: {:ok, t} | {:error, term}
  def acquire(opts) when is_list(opts) do
    with {:ok, conn} <- fetch_conn(opts),
         acc = %__MODULE__{conn: conn},
         {:ok, acc} <- maybe_reboot(acc, opts) do
      maybe_verify_who_am_i(acc, opts)
    end
  end

  @doc """
  Read the device's `WHO_AM_I` register.
  """
  @spec who_am_i(t) :: {:ok, who_am_i} | {:error, term}
  def who_am_i(%__MODULE__{} = acc) do
    with {:ok, <<id>>} <- Registers.read_who_am_i(acc) do
      {:ok, id}
    end
  end

  @doc """
  Refresh the internal trim registers from non-volatile memory by setting
  `CTRL_REG5.BOOT`. Blocks for #{@boot_delay_ms} ms to give the device time
  to finish the boot sequence before returning.
  """
  @spec reboot(t) :: {:ok, t} | {:error, term}
  def reboot(%__MODULE__{} = acc) do
    with {:ok, acc} <-
           Registers.update_ctrl_reg_5(acc, fn <<byte>> ->
             <<byte ||| 1 <<< @ctrl_reg_5_boot_bit>>
           end) do
      Process.sleep(@boot_delay_ms)
      {:ok, acc}
    end
  end

  @doc """
  Configure the accelerometer's operating mode, ODR, range, axis enables, and
  block-data-update setting. Caches the chosen `:operating_mode` and `:range`
  on the struct so subsequent reads can scale samples without re-reading the
  config registers.

  See `LIS3DH.Config.encode_ctrl_reg_1/1` and
  `LIS3DH.Config.encode_ctrl_reg_4/1` for the supported options. `:mode` and
  `:odr` are required.

  Writes `CTRL_REG4` first (range / HR / BDU), then `CTRL_REG1` (ODR / LPen /
  axes), so the device is fully reconfigured before sampling resumes.
  """
  @spec configure_accelerometer(t, keyword) :: {:ok, t} | {:error, term}
  def configure_accelerometer(%__MODULE__{} = acc, opts) when is_list(opts) do
    mode = Keyword.fetch!(opts, :mode)
    range = Keyword.get(opts, :range, 2)

    ctrl_reg_1 = Config.encode_ctrl_reg_1(opts)
    ctrl_reg_4 = Config.encode_ctrl_reg_4(opts)

    with {:ok, acc} <- Registers.write_ctrl_reg_4(acc, ctrl_reg_4),
         {:ok, acc} <- Registers.write_ctrl_reg_1(acc, ctrl_reg_1) do
      {:ok, %{acc | operating_mode: mode, range: range}}
    end
  end

  @doc """
  Populate the cached `:operating_mode` and `:range` by reading `CTRL_REG1`
  and `CTRL_REG4`. Useful after `acquire/1` when the device has already been
  configured by some other process.
  """
  @spec detect_configuration(t) :: {:ok, t} | {:error, term}
  def detect_configuration(%__MODULE__{} = acc) do
    with {:ok, ctrl_reg_1} <- Registers.read_ctrl_reg_1(acc),
         {:ok, ctrl_reg_4} <- Registers.read_ctrl_reg_4(acc) do
      %{lpen: lpen} = Config.decode_ctrl_reg_1(ctrl_reg_1)
      %{hr: hr, range: range} = Config.decode_ctrl_reg_4(ctrl_reg_4)
      mode = Config.operating_mode(lpen, hr)
      {:ok, %{acc | operating_mode: mode, range: range}}
    end
  end

  @doc """
  Read the accelerometer x/y/z sample and return scaled values in m/s².

  Requires `:operating_mode` and `:range` to be cached on the struct — call
  `configure_accelerometer/2` or `detect_configuration/1` first.
  """
  @spec read_accelerometer(t) :: {:ok, axes} | {:error, term}
  def read_accelerometer(%__MODULE__{operating_mode: nil}),
    do: {:error, :operating_mode_not_set}

  def read_accelerometer(%__MODULE__{range: nil}),
    do: {:error, :range_not_set}

  def read_accelerometer(%__MODULE__{operating_mode: mode, range: range} = acc) do
    with {:ok, <<x::little-signed-16, y::little-signed-16, z::little-signed-16>>} <-
           Chip.read_register(acc, @out_x_l, 6) do
      {:ok, %{x: scale(x, mode, range), y: scale(y, mode, range), z: scale(z, mode, range)}}
    end
  end

  @doc """
  Set `CTRL_REG1.ODR` to a non-zero rate without changing the other fields,
  bringing the sensor out of power-down. Equivalent to a write to `CTRL_REG1`
  with the chosen ODR while preserving the LPen and axis enable bits.
  """
  @spec power_on(t, Config.odr()) :: {:ok, t} | {:error, term}
  def power_on(%__MODULE__{} = acc, odr) do
    Registers.update_ctrl_reg_1(acc, fn <<byte>> ->
      odr_code = encode_odr!(odr)
      <<odr_code <<< 4 ||| (byte &&& 0x0F)>>
    end)
  end

  @doc """
  Set `CTRL_REG1.ODR` to `0000` (power-down mode), preserving the other
  fields.
  """
  @spec power_off(t) :: {:ok, t} | {:error, term}
  def power_off(%__MODULE__{} = acc) do
    Registers.update_ctrl_reg_1(acc, fn <<byte>> -> <<byte &&& 0x0F>> end)
  end

  @doc """
  Enable the on-chip auxiliary ADC by setting `TEMP_CFG_REG.ADC_EN`. The ADC
  samples at the configured `CTRL_REG1.ODR`. Requires `:block_data_update`
  (`CTRL_REG4.BDU`) to be `:hold` for consistent reads — `configure_accelerometer/2`
  defaults to that already.
  """
  @spec enable_auxiliary_adc(t) :: {:ok, t} | {:error, term}
  def enable_auxiliary_adc(%__MODULE__{} = acc) do
    Registers.update_temp_cfg_reg(acc, fn <<byte>> -> <<byte ||| 1 <<< @adc_en_bit>> end)
  end

  @doc "Clear `TEMP_CFG_REG.ADC_EN`, disabling all three auxiliary ADC channels."
  @spec disable_auxiliary_adc(t) :: {:ok, t} | {:error, term}
  def disable_auxiliary_adc(%__MODULE__{} = acc) do
    Registers.update_temp_cfg_reg(acc, fn <<byte>> ->
      <<byte &&& bnot(1 <<< @adc_en_bit) &&& 0xFF>>
    end)
  end

  @doc """
  Enable the embedded temperature sensor by setting both `TEMP_CFG_REG.ADC_EN`
  and `TEMP_CFG_REG.TEMP_EN`. The temperature reading is routed to channel 3
  of the auxiliary ADC; read it via `read_temperature/1`.
  """
  @spec enable_temperature_sensor(t) :: {:ok, t} | {:error, term}
  def enable_temperature_sensor(%__MODULE__{} = acc) do
    Registers.update_temp_cfg_reg(acc, fn <<_byte>> ->
      <<1 <<< @adc_en_bit ||| 1 <<< @temp_en_bit>>
    end)
  end

  @doc "Clear `TEMP_CFG_REG.TEMP_EN` (leaving `ADC_EN` alone)."
  @spec disable_temperature_sensor(t) :: {:ok, t} | {:error, term}
  def disable_temperature_sensor(%__MODULE__{} = acc) do
    Registers.update_temp_cfg_reg(acc, fn <<byte>> ->
      <<byte &&& bnot(1 <<< @temp_en_bit) &&& 0xFF>>
    end)
  end

  @doc """
  Read auxiliary ADC channel 1, 2, or 3 and return the absolute voltage in
  millivolts.

  The chip's ADC input range is centred on #{@aux_adc_center_mv} mV with a
  ±#{@aux_adc_span_mv} mV span, so the returned value is in
  `#{@aux_adc_center_mv - @aux_adc_span_mv}..#{@aux_adc_center_mv + @aux_adc_span_mv}` mV.
  ADC resolution depends on the operating mode (10-bit in normal /
  high-resolution, 8-bit in low-power), so this function requires
  `:operating_mode` to be cached on the struct.
  """
  @spec read_auxiliary_adc(t, 1 | 2 | 3) :: {:ok, float} | {:error, term}
  def read_auxiliary_adc(%__MODULE__{operating_mode: nil}, _channel),
    do: {:error, :operating_mode_not_set}

  def read_auxiliary_adc(%__MODULE__{operating_mode: mode} = acc, channel)
      when channel in 1..3 do
    address = @out_adc1_l + (channel - 1) * 2

    with {:ok, <<raw::little-signed-16>>} <- Chip.read_register(acc, address, 2) do
      {:ok, scale_aux_adc(raw, mode)}
    end
  end

  @doc """
  Read the embedded temperature sensor on auxiliary ADC channel 3 and return
  the **delta** temperature in °C, relative to the 25 °C factory
  calibration point (i.e. add `25.0` for the absolute reading).

  Only the `OUT_ADC3_H` byte carries temperature data — sensitivity is
  `1 LSB/°C` and resolution is 8-bit regardless of operating mode
  (datasheet §3.2). The full 16-bit word is still read so `BDU=:hold`
  unlatches cleanly.

  Requires the temperature sensor to be enabled via
  `enable_temperature_sensor/1`.
  """
  @spec read_temperature(t) :: {:ok, float} | {:error, term}
  def read_temperature(%__MODULE__{} = acc) do
    with {:ok, <<raw::little-signed-16>>} <- Chip.read_register(acc, @out_adc1_l + 4, 2) do
      {:ok, (raw >>> 8) * 1.0}
    end
  end

  # Data is left-justified — meaningful bits at the MSB end. Arithmetic right
  # shift recovers the native signed N-bit value, then we scale by the
  # per-mode mg/LSB and convert mg → m/s².
  defp scale(raw, mode, range) do
    shift = 16 - Config.native_width(mode)
    sensitivity_mg = Config.sensitivity(mode, range)
    (raw >>> shift) * sensitivity_mg * @gravity_ms2 / 1000
  end

  # Aux ADC is left-justified 10-bit (HR/Normal) or 8-bit (low-power) signed.
  # Recover the N-bit signed value, then map ±full-scale → ±@aux_adc_span_mv
  # added to the @aux_adc_center_mv centre.
  defp scale_aux_adc(raw, mode) do
    width = Config.aux_adc_width(mode)
    shift = 16 - width
    full_scale = 1 <<< (width - 1)
    @aux_adc_center_mv + (raw >>> shift) * @aux_adc_span_mv / full_scale
  end

  defp encode_odr!(odr) do
    Map.fetch!(
      %{
        :power_down => 0b0000,
        1 => 0b0001,
        10 => 0b0010,
        25 => 0b0011,
        50 => 0b0100,
        100 => 0b0101,
        200 => 0b0110,
        400 => 0b0111,
        1600 => 0b1000,
        1344 => 0b1001,
        5376 => 0b1001
      },
      odr
    )
  end

  @doc """
  Configure the on-chip high-pass filter via `CTRL_REG2`.

  See `LIS3DH.Config.encode_ctrl_reg_2/1` for the supported options.
  """
  @spec configure_high_pass_filter(t, keyword) :: {:ok, t} | {:error, term}
  def configure_high_pass_filter(%__MODULE__{} = acc, opts \\ []) do
    Registers.write_ctrl_reg_2(acc, Config.encode_ctrl_reg_2(opts))
  end

  @doc """
  Read the `REFERENCE` register. With `:normal_with_reset` HPF mode (the
  default after power-up), this read also resets the high-pass filter's
  internal state.
  """
  @spec read_reference(t) :: {:ok, integer} | {:error, term}
  def read_reference(%__MODULE__{} = acc) do
    with {:ok, <<value::signed-8>>} <- Registers.read_reference(acc) do
      {:ok, value}
    end
  end

  @doc "Write the `REFERENCE` register (used as the HPF reference in `:reference` mode)."
  @spec write_reference(t, integer) :: {:ok, t} | {:error, term}
  def write_reference(%__MODULE__{} = acc, value) when value in -128..127 do
    Registers.write_reference(acc, <<value::signed-8>>)
  end

  @doc """
  Configure a free-fall detector on the given interrupt pin.

  Free-fall is signalled when the magnitude of acceleration on all three
  axes falls below a threshold for a configurable duration (i.e. the device
  is in true free fall, ~0 g on every axis).

  ## Options

    * `:threshold_mg` — threshold in milli-g (default `350`, the AN3308
      recommended value). Lower thresholds trigger more easily.
    * `:duration` — `0..127` count of `1/ODR` periods (default `5`).
  """
  @spec configure_free_fall(t, Interrupts.pin(), keyword) :: {:ok, t} | {:error, term}
  def configure_free_fall(%__MODULE__{} = acc, pin, opts \\ []) do
    configure_inertial_interrupt(acc, pin,
      mode: :and,
      axes: [:x_low, :y_low, :z_low],
      threshold_mg: Keyword.get(opts, :threshold_mg, 350),
      duration: Keyword.get(opts, :duration, 5)
    )
  end

  @doc """
  Configure a motion (wake-up) detector on the given interrupt pin.

  Motion is signalled when **any** enabled axis exceeds the threshold for
  the configured duration.

  ## Options

    * `:threshold_mg` — threshold in milli-g (no default, must be specified).
    * `:duration` — `0..127` count of `1/ODR` periods (default `0`).
    * `:axes` — list of `t:LIS3DH.Interrupts.axis_event/0` (default
      `[:x_high, :y_high, :z_high]`).
  """
  @spec configure_motion(t, Interrupts.pin(), keyword) :: {:ok, t} | {:error, term}
  def configure_motion(%__MODULE__{} = acc, pin, opts) do
    configure_inertial_interrupt(acc, pin,
      mode: :or,
      axes: Keyword.get(opts, :axes, [:x_high, :y_high, :z_high]),
      threshold_mg: Keyword.fetch!(opts, :threshold_mg),
      duration: Keyword.get(opts, :duration, 0)
    )
  end

  @doc """
  Configure 6D or 4D orientation detection on the given interrupt pin.

  ## Options

    * `:mode` — `:movement` (interrupt fires on transitions between known
      zones) or `:position` (interrupt stays asserted while inside a known
      zone). Default `:position`.
    * `:detection` — `:six_d` (default, all six face-down/face-up directions)
      or `:four_d` (X/Y plane only, Z ignored — for portrait/landscape).
    * `:axes` — list of `t:LIS3DH.Interrupts.axis_event/0` to enable
      (default all six).
    * `:threshold_mg` — threshold in milli-g (no default; the zone half-width
      is typically chosen so two zones don't overlap).
    * `:duration` — `0..127` count of `1/ODR` periods (default `0`).

  Writes the configured `INT*_CFG`, `INT*_THS`, `INT*_DURATION` and also
  toggles `CTRL_REG5.D4D_INT*` to match the `:detection` choice.
  """
  @spec configure_orientation(t, Interrupts.pin(), keyword) :: {:ok, t} | {:error, term}
  def configure_orientation(%__MODULE__{} = acc, pin, opts) do
    aoi_mode =
      case Keyword.get(opts, :mode, :position) do
        :movement -> :six_d_movement
        :position -> :six_d_position
      end

    detection = Keyword.get(opts, :detection, :six_d)

    axes =
      Keyword.get(opts, :axes, [:x_high, :x_low, :y_high, :y_low, :z_high, :z_low])

    with {:ok, acc} <-
           configure_inertial_interrupt(acc, pin,
             mode: aoi_mode,
             axes: axes,
             threshold_mg: Keyword.fetch!(opts, :threshold_mg),
             duration: Keyword.get(opts, :duration, 0)
           ) do
      set_4d_detection(acc, pin, detection == :four_d)
    end
  end

  @doc """
  Configure sleep-to-wake / return-to-sleep by writing `ACT_THS` and
  `ACT_DUR`.

  When acceleration falls below `:threshold_mg` for the configured
  `:duration`, the device automatically switches to low-power mode at 10 Hz
  ODR regardless of the original `CTRL_REG1` / `CTRL_REG4` settings. When
  acceleration rises above the threshold, the device restores the original
  configuration.

  ## Options

    * `:threshold_mg` — threshold in milli-g (required). Uses the same LSB
      table as `INT*_THS`. Pass `0` to disable activity detection.
    * `:duration` — `0..255` (required). One LSB corresponds to
      `(8 × duration + 1) / ODR` seconds per datasheet §8.36.

  Requires the accelerometer range to be cached on the struct.
  """
  @spec configure_activity(t, keyword) :: {:ok, t} | {:error, term}
  def configure_activity(%__MODULE__{range: nil}, _opts), do: {:error, :range_not_set}

  def configure_activity(%__MODULE__{range: range} = acc, opts) do
    threshold_mg = Keyword.fetch!(opts, :threshold_mg)
    duration = Keyword.fetch!(opts, :duration)

    unless is_integer(duration) and duration in 0..255,
      do: raise(ArgumentError, "invalid duration: #{inspect(duration)} (must be 0..255)")

    ths = Interrupts.encode_threshold!(threshold_mg, range)

    with {:ok, acc} <- Registers.write_act_ths(acc, ths) do
      Registers.write_act_dur(acc, <<duration>>)
    end
  end

  @doc "Disable activity detection by writing `0` to `ACT_THS`."
  @spec disable_activity(t) :: {:ok, t} | {:error, term}
  def disable_activity(%__MODULE__{} = acc) do
    Registers.write_act_ths(acc, <<0>>)
  end

  @doc """
  Configure click / double-click / tap detection by writing `CLICK_CFG`,
  `CLICK_THS`, `TIME_LIMIT`, `TIME_LATENCY`, and `TIME_WINDOW`.

  ## Options

    * `:events` — list of `t:LIS3DH.Click.click_event/0` to enable
      (required; pass `[]` to disable all).
    * `:threshold_mg` — threshold in milli-g (required). Same LSB table as
      `INT*_THS`.
    * `:latched` — when `true`, the click interrupt stays high until
      `CLICK_SRC` is read (default `false`).
    * `:time_limit` — `0..127` count of `1/ODR` periods, the max click pulse
      width (required).
    * `:time_latency` — `0..255` count of `1/ODR` periods, the dead time
      after a click (required).
    * `:time_window` — `0..255` count of `1/ODR` periods, the search window
      for the second click of a double-click (default `0`).

  Requires the accelerometer range to be cached on the struct.
  """
  @spec configure_click(t, keyword) :: {:ok, t} | {:error, term}
  def configure_click(%__MODULE__{range: nil}, _opts), do: {:error, :range_not_set}

  def configure_click(%__MODULE__{range: range} = acc, opts) do
    events = Keyword.fetch!(opts, :events)
    threshold_mg = Keyword.fetch!(opts, :threshold_mg)
    latched = Keyword.get(opts, :latched, false)
    time_limit = Keyword.fetch!(opts, :time_limit)
    time_latency = Keyword.fetch!(opts, :time_latency)
    time_window = Keyword.get(opts, :time_window, 0)

    unless is_integer(time_limit) and time_limit in 0..127,
      do: raise(ArgumentError, "invalid time_limit: #{inspect(time_limit)} (must be 0..127)")

    unless is_integer(time_latency) and time_latency in 0..255,
      do: raise(ArgumentError, "invalid time_latency: #{inspect(time_latency)} (must be 0..255)")

    unless is_integer(time_window) and time_window in 0..255,
      do: raise(ArgumentError, "invalid time_window: #{inspect(time_window)} (must be 0..255)")

    with {:ok, acc} <-
           Registers.write_click_ths(acc, Click.encode_click_ths!(threshold_mg, range, latched)),
         {:ok, acc} <- Registers.write_time_limit(acc, <<time_limit>>),
         {:ok, acc} <- Registers.write_time_latency(acc, <<time_latency>>),
         {:ok, acc} <- Registers.write_time_window(acc, <<time_window>>) do
      Registers.write_click_cfg(acc, Click.encode_click_cfg(events))
    end
  end

  @doc """
  Read the `CLICK_SRC` register and decode it. Reading clears the latched
  flags if `LIR_Click` was set during configure.
  """
  @spec read_click_source(t) :: {:ok, Click.source_flags()} | {:error, term}
  def read_click_source(%__MODULE__{} = acc) do
    with {:ok, byte} <- Registers.read_click_src(acc) do
      {:ok, Click.decode_click_src(byte)}
    end
  end

  @doc """
  Configure an inertial interrupt (1 or 2) by writing `INT*_CFG`, `INT*_THS`,
  and `INT*_DURATION` atomically.

  ## Options

    * `:mode` — `t:LIS3DH.Interrupts.aoi_mode/0` (default `:or`).
    * `:axes` — list of `t:LIS3DH.Interrupts.axis_event/0` to enable.
    * `:threshold_mg` — non-negative integer threshold in milli-g. The
      LSB size depends on the cached `:range`; this function reads the
      cached value and rounds the threshold to fit.
    * `:duration` — `0..127` count of `1/ODR` periods the condition must
      hold before the interrupt fires (default `0`).

  Requires the accelerometer range to be cached on the struct.
  """
  @spec configure_inertial_interrupt(t, Interrupts.pin(), keyword) :: {:ok, t} | {:error, term}
  def configure_inertial_interrupt(%__MODULE__{range: nil}, _pin, _opts),
    do: {:error, :range_not_set}

  def configure_inertial_interrupt(%__MODULE__{range: range} = acc, pin, opts)
      when pin in [:int1, :int2] do
    cfg = Interrupts.encode_int_cfg(opts)
    ths = Interrupts.encode_threshold!(Keyword.get(opts, :threshold_mg, 0), range)
    dur = Interrupts.encode_duration!(Keyword.get(opts, :duration, 0))

    {cfg_w, ths_w, dur_w} =
      case pin do
        :int1 ->
          {&Registers.write_int1_cfg/2, &Registers.write_int1_ths/2,
           &Registers.write_int1_duration/2}

        :int2 ->
          {&Registers.write_int2_cfg/2, &Registers.write_int2_ths/2,
           &Registers.write_int2_duration/2}
      end

    with {:ok, acc} <- ths_w.(acc, ths),
         {:ok, acc} <- dur_w.(acc, dur) do
      cfg_w.(acc, cfg)
    end
  end

  @doc """
  Read the `INT*_SRC` register. Reading clears the latched flags if latching
  is enabled (`LIR_INTx` in `CTRL_REG5`).
  """
  @spec read_interrupt_source(t, Interrupts.pin()) ::
          {:ok, Interrupts.source_flags()} | {:error, term}
  def read_interrupt_source(%__MODULE__{} = acc, pin) when pin in [:int1, :int2] do
    reader = if pin == :int1, do: &Registers.read_int1_src/1, else: &Registers.read_int2_src/1

    with {:ok, byte} <- reader.(acc) do
      {:ok, Interrupts.decode_int_src(byte)}
    end
  end

  @doc """
  OR-in the given routing bits in `CTRL_REG3` (INT1 routing). Leaves the
  other bits untouched, so it composes cleanly with `LIS3DH.Sampler` which
  also writes the FIFO bits in this register.

  Valid `events`: `:click`, `:ia1`, `:ia2`, `:zyxda`, `:adc_drdy_321`,
  `:fifo_watermark`, `:fifo_overrun`.
  """
  @spec enable_int1_routing(t, [int1_event]) :: {:ok, t} | {:error, term}
        when int1_event:
               :click | :ia1 | :ia2 | :zyxda | :adc_drdy_321 | :fifo_watermark | :fifo_overrun
  def enable_int1_routing(%__MODULE__{} = acc, events) when is_list(events) do
    mask = int1_routing_mask(events)
    Registers.update_ctrl_reg_3(acc, fn <<byte>> -> <<byte ||| mask>> end)
  end

  @doc "Mask out the given routing bits in `CTRL_REG3` (INT1 routing)."
  @spec disable_int1_routing(t, [int1_event]) :: {:ok, t} | {:error, term}
        when int1_event:
               :click | :ia1 | :ia2 | :zyxda | :adc_drdy_321 | :fifo_watermark | :fifo_overrun
  def disable_int1_routing(%__MODULE__{} = acc, events) when is_list(events) do
    mask = int1_routing_mask(events)
    Registers.update_ctrl_reg_3(acc, fn <<byte>> -> <<byte &&& bnot(mask) &&& 0xFF>> end)
  end

  @doc """
  OR-in the given routing bits in `CTRL_REG6` (INT2 routing). Preserves the
  `INT_POLARITY` bit and any others not in `events`.

  Valid `events`: `:click`, `:ia1`, `:ia2`, `:boot`, `:activity`.
  """
  @spec enable_int2_routing(t, [int2_event]) :: {:ok, t} | {:error, term}
        when int2_event: :click | :ia1 | :ia2 | :boot | :activity
  def enable_int2_routing(%__MODULE__{} = acc, events) when is_list(events) do
    mask = int2_routing_mask(events)
    Registers.update_ctrl_reg_6(acc, fn <<byte>> -> <<byte ||| mask>> end)
  end

  @doc "Mask out the given routing bits in `CTRL_REG6` (INT2 routing)."
  @spec disable_int2_routing(t, [int2_event]) :: {:ok, t} | {:error, term}
        when int2_event: :click | :ia1 | :ia2 | :boot | :activity
  def disable_int2_routing(%__MODULE__{} = acc, events) when is_list(events) do
    mask = int2_routing_mask(events)
    Registers.update_ctrl_reg_6(acc, fn <<byte>> -> <<byte &&& bnot(mask) &&& 0xFF>> end)
  end

  @doc """
  Set the active level for both INT pins via `CTRL_REG6.INT_POLARITY`.

  `polarity` is `:active_high` (default after reset) or `:active_low`.
  """
  @spec set_interrupt_polarity(t, :active_high | :active_low) :: {:ok, t} | {:error, term}
  def set_interrupt_polarity(%__MODULE__{} = acc, polarity)
      when polarity in [:active_high, :active_low] do
    bit = if polarity == :active_low, do: 1 <<< 1, else: 0

    Registers.update_ctrl_reg_6(acc, fn <<byte>> ->
      <<(byte &&& bnot(1 <<< 1) &&& 0xFF) ||| bit>>
    end)
  end

  @doc """
  Toggle interrupt latching for the given pin via `CTRL_REG5.LIR_INT1` /
  `LIR_INT2`. When latched, the interrupt pin stays asserted until the
  corresponding `INT*_SRC` register is read.
  """
  @spec set_interrupt_latching(t, Interrupts.pin(), boolean) :: {:ok, t} | {:error, term}
  def set_interrupt_latching(%__MODULE__{} = acc, pin, latched?) when pin in [:int1, :int2] do
    bit = if pin == :int1, do: 3, else: 1
    update_bit(acc, &Registers.update_ctrl_reg_5/2, bit, latched?)
  end

  @doc """
  Toggle 4D detection for the given pin via `CTRL_REG5.D4D_INT1` /
  `D4D_INT2`. 4D restricts 6D detection to the X/Y plane (Z position
  ignored). Has no effect unless `INT*_CFG.6D` is also set.
  """
  @spec set_4d_detection(t, Interrupts.pin(), boolean) :: {:ok, t} | {:error, term}
  def set_4d_detection(%__MODULE__{} = acc, pin, enabled?) when pin in [:int1, :int2] do
    bit = if pin == :int1, do: 2, else: 0
    update_bit(acc, &Registers.update_ctrl_reg_5/2, bit, enabled?)
  end

  defp int1_routing_mask(events) do
    map = %{
      click: 1 <<< 7,
      ia1: 1 <<< 6,
      ia2: 1 <<< 5,
      zyxda: 1 <<< 4,
      adc_drdy_321: 1 <<< 3,
      fifo_watermark: 1 <<< 2,
      fifo_overrun: 1 <<< 1
    }

    Enum.reduce(events, 0, fn event, acc ->
      acc ||| Map.fetch!(map, event)
    end)
  end

  defp int2_routing_mask(events) do
    map = %{
      click: 1 <<< 7,
      ia1: 1 <<< 6,
      ia2: 1 <<< 5,
      boot: 1 <<< 4,
      activity: 1 <<< 3
    }

    Enum.reduce(events, 0, fn event, acc ->
      acc ||| Map.fetch!(map, event)
    end)
  end

  defp update_bit(acc, updater, bit, true),
    do: updater.(acc, fn <<byte>> -> <<byte ||| 1 <<< bit>> end)

  defp update_bit(acc, updater, bit, false),
    do: updater.(acc, fn <<byte>> -> <<byte &&& bnot(1 <<< bit) &&& 0xFF>> end)

  @doc """
  Set the `CTRL_REG4.ST` self-test field while preserving the other bits.

  The recommended self-test procedure (per ST application note AN3308) is:

    1. Power up the device and `configure_accelerometer/2` for normal mode,
       ±2g, 50 Hz, BDU=`:hold`.
    2. Wait for stable output (≥ a few ODR periods) and average several
       baseline samples.
    3. Call `set_self_test(acc, :self_test_0)` and wait for the documented
       turn-on time (90 ms typical).
    4. Average several test samples; the per-axis delta vs. the baseline
       must fall within the limits in datasheet table 4.
    5. Restore with `set_self_test(acc, :off)`.
    6. Optionally repeat with `:self_test_1` for the alternate direction.

  This helper just toggles the ST field; the user owns the timing,
  averaging, and pass/fail check.
  """
  @spec set_self_test(t, Config.self_test_mode()) :: {:ok, t} | {:error, term}
  def set_self_test(%__MODULE__{} = acc, mode) do
    st_code = Config.self_test_code(mode)

    Registers.update_ctrl_reg_4(acc, fn <<byte>> ->
      <<(byte &&& bnot(0b110) &&& 0xFF) ||| st_code <<< 1>>
    end)
  end

  defp fetch_conn(opts) do
    case Keyword.fetch(opts, :conn) do
      {:ok, conn} -> {:ok, conn}
      :error -> {:error, "`:conn` option is required"}
    end
  end

  defp maybe_reboot(acc, opts) do
    if Keyword.get(opts, :reboot, false), do: reboot(acc), else: {:ok, acc}
  end

  defp maybe_verify_who_am_i(acc, opts) do
    if Keyword.get(opts, :verify_who_am_i, true), do: verify_who_am_i(acc), else: {:ok, acc}
  end

  defp verify_who_am_i(acc) do
    case who_am_i(acc) do
      {:ok, @expected_who_am_i} -> {:ok, acc}
      {:ok, got} -> {:error, {:who_am_i_mismatch, got: got, expected: @expected_who_am_i}}
      {:error, _} = error -> error
    end
  end
end

defimpl Wafer.Chip, for: LIS3DH do
  @moduledoc """
  `Wafer.Chip` implementation that sets bit 7 (auto-increment) of the
  sub-address on every read and write, satisfying the LIS3DH's requirement
  for multi-byte transfers without breaking single-byte access.
  """

  import Bitwise

  alias Wafer.I2C

  @auto_increment 0x80

  def read_register(%LIS3DH{conn: inner}, address, bytes)
      when is_integer(address) and address in 0..0x7F and
             is_integer(bytes) and bytes > 0 do
    with {:ok, data, _inner} <-
           I2C.write_read(inner, <<address ||| @auto_increment>>, bytes, []) do
      {:ok, data}
    end
  end

  def read_register(_conn, address, bytes) do
    {:error,
     "Invalid argument: address=#{inspect(address)} bytes=#{inspect(bytes)} " <>
       "(address must be in 0..0x7F, bytes must be a positive integer)"}
  end

  def write_register(%LIS3DH{conn: inner} = conn, address, data)
      when is_integer(address) and address in 0..0x7F and
             is_binary(data) and byte_size(data) > 0 do
    with {:ok, inner} <-
           I2C.write(inner, <<address ||| @auto_increment, data::binary>>, []) do
      {:ok, %{conn | conn: inner}}
    end
  end

  def write_register(_conn, address, data) do
    {:error,
     "Invalid argument: address=#{inspect(address)} data=#{inspect(data)} " <>
       "(address must be in 0..0x7F, data must be a non-empty binary)"}
  end

  def swap_register(conn, address, data) when is_binary(data) do
    with {:ok, old} <- read_register(conn, address, byte_size(data)),
         {:ok, conn} <- write_register(conn, address, data) do
      {:ok, old, conn}
    end
  end

  def swap_register(_conn, _address, data),
    do: {:error, "Invalid argument: data must be a binary, got #{inspect(data)}"}
end
