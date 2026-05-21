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

  alias LIS3DH.Config
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
           Registers.update_ctrl_reg_5(acc, fn <<byte>> -> <<byte ||| 1 <<< @ctrl_reg_5_boot_bit>> end) do
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
    Registers.update_temp_cfg_reg(acc, fn <<byte>> -> <<byte &&& bnot(1 <<< @adc_en_bit) &&& 0xFF>> end)
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
    Registers.update_temp_cfg_reg(acc, fn <<byte>> -> <<byte &&& bnot(1 <<< @temp_en_bit) &&& 0xFF>> end)
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
  the **delta** temperature in °C.

  The LIS3DH's temperature sensor outputs `1 digit/°C` relative to an
  unspecified factory reference (often approximated as ambient at
  calibration time). Add your own reference offset (commonly `25.0`) for an
  absolute reading.

  Requires the temperature sensor to be enabled via
  `enable_temperature_sensor/1` and `:operating_mode` to be cached.
  """
  @spec read_temperature(t) :: {:ok, float} | {:error, term}
  def read_temperature(%__MODULE__{operating_mode: nil}),
    do: {:error, :operating_mode_not_set}

  def read_temperature(%__MODULE__{operating_mode: mode} = acc) do
    with {:ok, <<raw::little-signed-16>>} <- Chip.read_register(acc, @out_adc1_l + 4, 2) do
      shift = 16 - Config.aux_adc_width(mode)
      {:ok, (raw >>> shift) * 1.0}
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
      <<byte &&& bnot(0b110) &&& 0xFF ||| st_code <<< 1>>
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
