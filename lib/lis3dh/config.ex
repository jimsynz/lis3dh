defmodule LIS3DH.Config do
  @moduledoc """
  Encoding and decoding of the LIS3DH's `CTRL_REG1` (0x20) and `CTRL_REG4`
  (0x23) bytes, plus the per-mode/per-range sensitivity table used for
  converting raw ADC samples into physical units.

  Bit layouts:

  ```text
  CTRL_REG1 (0x20)
  | 7 6 5 4 | 3    | 2   1   0   |
  |   ODR   | LPen | Zen Yen Xen |

  CTRL_REG4 (0x23)
  | 7   | 6   | 5 4 | 3  | 2  1 | 0   |
  | BDU | BLE | FS  | HR |  ST  | SIM |
  ```

  Operating mode is set by the combination of `CTRL_REG1.LPen` and
  `CTRL_REG4.HR`:

  ```text
  LPen  HR  Mode               Data width
   1    0   Low-power           8-bit
   0    0   Normal              10-bit
   0    1  High-resolution     12-bit
   1    1   (Not allowed)
  ```

  All `OUT_*` data is signed 16-bit two's complement **left-justified** — the
  meaningful bits occupy the MSB end and the lower bits are zero. Use
  `sensitivity/2` to obtain the per-mode mg/LSB conversion factor.
  """

  import Bitwise

  @typedoc """
  Operating mode. Selects the ADC bit width and the LPen / HR bit
  combination.
  """
  @type operating_mode :: :low_power | :normal | :high_resolution

  @typedoc "Output data rate in Hz, or `:power_down` to disable the sensor."
  @type odr ::
          :power_down
          | 1
          | 10
          | 25
          | 50
          | 100
          | 200
          | 400
          | 1344
          | 1600
          | 5376

  @typedoc "Full-scale measurement range in g."
  @type range :: 2 | 4 | 8 | 16

  @typedoc "Axes to enable in `CTRL_REG1`."
  @type axis :: :x | :y | :z

  @typedoc "Block-data-update mode for `CTRL_REG4.BDU`."
  @type bdu :: :continuous | :hold

  @typedoc """
  High-pass filter mode for `CTRL_REG2.HPM`.

    * `:normal_with_reset` — continuous HPF; the internal state can be reset
      by reading the `REFERENCE` register.
    * `:reference` — HPF uses the `REFERENCE` register as the filtered
      reference signal.
    * `:normal` — continuous HPF with no reset hook.
    * `:autoreset` — HPF auto-resets on every interrupt event.
  """
  @type hpf_mode :: :normal_with_reset | :reference | :normal | :autoreset

  @typedoc "High-pass filter cutoff selector. The actual −3 dB cutoff depends on ODR; lower codes give higher cutoffs."
  @type hpf_cutoff :: 0..3

  @typedoc "Self-test mode selection for `CTRL_REG4.ST`."
  @type self_test_mode :: :off | :self_test_0 | :self_test_1

  @odr_codes %{
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
  }

  @range_codes %{2 => 0b00, 4 => 0b01, 8 => 0b10, 16 => 0b11}
  @bdu_codes %{continuous: 0, hold: 1}
  @hpf_mode_codes %{normal_with_reset: 0b00, reference: 0b01, normal: 0b10, autoreset: 0b11}
  @self_test_codes %{off: 0b00, self_test_0: 0b01, self_test_1: 0b10}

  # mg/LSB at the native bit width per (operating_mode, range).
  # Source: ST AN3308 application note / datasheet table 10 extended for all
  # ranges. The 16g column is non-linear by the chip's design.
  @sensitivities %{
    {:low_power, 2} => 16,
    {:low_power, 4} => 32,
    {:low_power, 8} => 64,
    {:low_power, 16} => 192,
    {:normal, 2} => 4,
    {:normal, 4} => 8,
    {:normal, 8} => 16,
    {:normal, 16} => 48,
    {:high_resolution, 2} => 1,
    {:high_resolution, 4} => 2,
    {:high_resolution, 8} => 4,
    {:high_resolution, 16} => 12
  }

  # Native bit widths per operating mode — equal to the right-shift required
  # to recover the N-bit ADC value from the 16-bit left-justified register.
  @native_widths %{low_power: 8, normal: 10, high_resolution: 12}

  @odr_decodes Map.new(Enum.reject(@odr_codes, fn {k, _} -> k == 5376 end), fn {k, v} ->
                 {v, k}
               end)
  @range_decodes Map.new(@range_codes, fn {k, v} -> {v, k} end)
  @bdu_decodes Map.new(@bdu_codes, fn {k, v} -> {v, k} end)
  @hpf_mode_decodes Map.new(@hpf_mode_codes, fn {k, v} -> {v, k} end)

  @doc """
  Encode a `CTRL_REG1` byte from keyword options.

  ## Options

    * `:mode` — `t:operating_mode/0` (required). Sets the `LPen` bit.
    * `:odr` — `t:odr/0` (required).
    * `:axes` — list of `t:axis/0` to enable (default `[:x, :y, :z]`).
  """
  @spec encode_ctrl_reg_1(keyword) :: <<_::8>>
  def encode_ctrl_reg_1(opts) when is_list(opts) do
    mode = Keyword.fetch!(opts, :mode)
    odr_code = lookup!(@odr_codes, Keyword.fetch!(opts, :odr), :odr)
    axes = Keyword.get(opts, :axes, [:x, :y, :z])

    lpen = if mode == :low_power, do: 1, else: 0
    zen = if :z in axes, do: 1, else: 0
    yen = if :y in axes, do: 1, else: 0
    xen = if :x in axes, do: 1, else: 0

    <<odr_code <<< 4 ||| lpen <<< 3 ||| zen <<< 2 ||| yen <<< 1 ||| xen>>
  end

  @doc """
  Encode a `CTRL_REG4` byte from keyword options. Leaves `BLE`, `ST`, and `SIM`
  at their reset values; callers that need to override them should compose
  the resulting binary with their own bit twiddling.

  ## Options

    * `:mode` — `t:operating_mode/0` (required). Sets the `HR` bit.
    * `:range` — `t:range/0` (default `2`).
    * `:block_data_update` — `t:bdu/0` (default `:hold`, recommended to avoid
      reading LSB/MSB pairs from different samples).
  """
  @spec encode_ctrl_reg_4(keyword) :: <<_::8>>
  def encode_ctrl_reg_4(opts) when is_list(opts) do
    mode = Keyword.fetch!(opts, :mode)
    range_code = lookup!(@range_codes, Keyword.get(opts, :range, 2), :range)
    bdu = lookup!(@bdu_codes, Keyword.get(opts, :block_data_update, :hold), :block_data_update)

    hr = if mode == :high_resolution, do: 1, else: 0

    <<bdu <<< 7 ||| range_code <<< 4 ||| hr <<< 3>>
  end

  @doc """
  Decode a `CTRL_REG1` byte into a map of its fields.

  Note that the `LPen` bit alone doesn't fully determine the operating mode —
  the `HR` bit in `CTRL_REG4` is also needed. This function reports `:lpen`
  as a boolean; combine it with `decode_ctrl_reg_4/1` to recover the full
  `t:operating_mode/0`.
  """
  @spec decode_ctrl_reg_1(<<_::8>>) :: %{lpen: boolean, odr: odr, axes: [axis]}
  def decode_ctrl_reg_1(<<byte>>) do
    %{
      lpen: (byte >>> 3 &&& 1) == 1,
      odr: lookup!(@odr_decodes, byte >>> 4 &&& 0b1111, :odr_code),
      axes: decode_axes(byte)
    }
  end

  @doc """
  Encode a `CTRL_REG2` byte (high-pass filter configuration) from keyword
  options.

  ## Options

    * `:mode` — `t:hpf_mode/0` (default `:normal_with_reset`).
    * `:cutoff` — `t:hpf_cutoff/0` (default `0`). The actual −3 dB cutoff
      depends on ODR per the datasheet figures.
    * `:filtered_data_output` (default `false`) — when `true`, the HPF output
      replaces the unfiltered data in `OUT_*` and the FIFO. When `false`
      (`FDS=0`) the HPF only affects the click / interrupt detectors.
    * `:enable_for_click` (default `false`) — apply HPF to the click
      detector.
    * `:enable_for_interrupt_1` (default `false`) — apply HPF to inertial
      interrupt 1 (AOI 1).
    * `:enable_for_interrupt_2` (default `false`) — apply HPF to inertial
      interrupt 2 (AOI 2).
  """
  @spec encode_ctrl_reg_2(keyword) :: <<_::8>>
  def encode_ctrl_reg_2(opts \\ []) when is_list(opts) do
    mode = lookup!(@hpf_mode_codes, Keyword.get(opts, :mode, :normal_with_reset), :mode)
    cutoff = Keyword.get(opts, :cutoff, 0)

    unless is_integer(cutoff) and cutoff in 0..3 do
      raise ArgumentError, "invalid cutoff: #{inspect(cutoff)} (valid values: 0..3)"
    end

    fds = if Keyword.get(opts, :filtered_data_output, false), do: 1, else: 0
    hpclick = if Keyword.get(opts, :enable_for_click, false), do: 1, else: 0
    hp_ia2 = if Keyword.get(opts, :enable_for_interrupt_2, false), do: 1, else: 0
    hp_ia1 = if Keyword.get(opts, :enable_for_interrupt_1, false), do: 1, else: 0

    <<mode <<< 6 ||| cutoff <<< 4 ||| fds <<< 3 ||| hpclick <<< 2 ||| hp_ia2 <<< 1 ||| hp_ia1>>
  end

  @doc "Decode a `CTRL_REG2` byte into a map of its fields."
  @spec decode_ctrl_reg_2(<<_::8>>) :: %{
          mode: hpf_mode,
          cutoff: hpf_cutoff,
          filtered_data_output: boolean,
          enable_for_click: boolean,
          enable_for_interrupt_1: boolean,
          enable_for_interrupt_2: boolean
        }
  def decode_ctrl_reg_2(<<byte>>) do
    %{
      mode: lookup!(@hpf_mode_decodes, byte >>> 6 &&& 0b11, :hpf_mode_code),
      cutoff: byte >>> 4 &&& 0b11,
      filtered_data_output: (byte >>> 3 &&& 1) == 1,
      enable_for_click: (byte >>> 2 &&& 1) == 1,
      enable_for_interrupt_2: (byte >>> 1 &&& 1) == 1,
      enable_for_interrupt_1: (byte &&& 1) == 1
    }
  end

  @doc "Decode a `CTRL_REG4` byte into a map of its fields."
  @spec decode_ctrl_reg_4(<<_::8>>) :: %{hr: boolean, range: range, block_data_update: bdu}
  def decode_ctrl_reg_4(<<byte>>) do
    %{
      hr: (byte >>> 3 &&& 1) == 1,
      range: lookup!(@range_decodes, byte >>> 4 &&& 0b11, :range_code),
      block_data_update: lookup!(@bdu_decodes, byte >>> 7 &&& 1, :bdu_code)
    }
  end

  @doc """
  Resolve the operating mode from the `LPen` and `HR` bits returned by
  `decode_ctrl_reg_1/1` and `decode_ctrl_reg_4/1`.

  Raises `ArgumentError` for the disallowed combination LPen=1, HR=1.
  """
  @spec operating_mode(boolean, boolean) :: operating_mode
  def operating_mode(false, false), do: :normal
  def operating_mode(true, false), do: :low_power
  def operating_mode(false, true), do: :high_resolution

  def operating_mode(true, true) do
    raise ArgumentError,
          "invalid operating mode combination: LPen=1 + HR=1 is not allowed"
  end

  @doc """
  Encode a self-test mode atom to its 2-bit `CTRL_REG4.ST` code.
  """
  @spec self_test_code(self_test_mode) :: 0..2
  def self_test_code(mode), do: lookup!(@self_test_codes, mode, :self_test_mode)

  @doc """
  Returns the per-LSB sensitivity in milli-g at the native bit width for the
  given operating mode and range.
  """
  @spec sensitivity(operating_mode, range) :: pos_integer
  def sensitivity(mode, range), do: Map.fetch!(@sensitivities, {mode, range})

  @doc """
  Returns the native accelerometer ADC bit width for the given operating
  mode — equivalent to the right-shift required to recover the N-bit signed
  value from the 16-bit left-justified `OUT_*` registers.
  """
  @spec native_width(operating_mode) :: 8 | 10 | 12
  def native_width(mode), do: Map.fetch!(@native_widths, mode)

  @doc """
  Returns the bit width of the auxiliary ADC for the given operating mode.

  Per datasheet §3.7, the auxiliary ADC is 8-bit when `LPen=1` (low-power)
  and 10-bit otherwise — including in high-resolution mode, even though the
  accelerometer itself is 12-bit there.
  """
  @spec aux_adc_width(operating_mode) :: 8 | 10
  def aux_adc_width(:low_power), do: 8
  def aux_adc_width(_mode), do: 10

  defp decode_axes(byte) do
    Enum.filter([:x, :y, :z], fn axis ->
      bit = %{x: 0, y: 1, z: 2}[axis]
      (byte >>> bit &&& 1) == 1
    end)
  end

  defp lookup!(map, key, field) do
    case Map.fetch(map, key) do
      {:ok, value} ->
        value

      :error ->
        raise ArgumentError,
              "invalid #{field}: #{inspect(key)} (valid values: #{inspect(Map.keys(map))})"
    end
  end
end
