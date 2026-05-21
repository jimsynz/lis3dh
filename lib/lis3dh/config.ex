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
  Returns the per-LSB sensitivity in milli-g at the native bit width for the
  given operating mode and range.
  """
  @spec sensitivity(operating_mode, range) :: pos_integer
  def sensitivity(mode, range), do: Map.fetch!(@sensitivities, {mode, range})

  @doc """
  Returns the native ADC bit width for the given operating mode — equivalent
  to the right-shift required to recover the N-bit signed value from the
  16-bit left-justified `OUT_*` registers.
  """
  @spec native_width(operating_mode) :: 8 | 10 | 12
  def native_width(mode), do: Map.fetch!(@native_widths, mode)

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
