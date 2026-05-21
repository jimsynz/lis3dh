defmodule LIS3DH.Interrupts do
  @moduledoc """
  Encoding and decoding for the LIS3DH's inertial interrupt configuration
  (`INT1_CFG`, `INT1_THS`, `INT1_DURATION` and their `INT2_*` siblings) plus
  the per-pin routing bits in `CTRL_REG3` and `CTRL_REG6` and the
  latching/4D bits in `CTRL_REG5`.

  The chip has two physical interrupt pins, INT1 and INT2, each driven by a
  configurable mix of event sources:

    * inertial interrupts 1 / 2 (AOI engines reading `INT*_CFG`),
    * click detection,
    * data-ready for accelerometer (`ZYXDA`) or auxiliary ADC (`321DA`),
    * FIFO watermark / overrun (INT1 only — handled by `LIS3DH.Sampler`),
    * activity / boot (INT2 only).

  An inertial interrupt fires when the per-axis event flags in `INT*_CFG`
  combine according to the `AOI` / `6D` bits:

  ```text
  AOI  6D   Behaviour
   0   0   OR of the enabled axis events (e.g. wake-up / motion)
   0   1   6D movement recognition (entering a known zone)
   1   0   AND of the enabled axis events (e.g. free-fall)
   1   1   6D position recognition (currently in a known zone)
  ```

  Threshold and duration registers carry units that depend on the configured
  full-scale range and ODR; see `threshold_lsb_mg/1` and the helpers in
  `LIS3DH` for unit-aware wrappers.
  """

  import Bitwise

  alias LIS3DH.Config

  @typedoc "Which interrupt pin a configuration applies to."
  @type pin :: :int1 | :int2

  @typedoc """
  Combined AOI/6D field in `INT*_CFG`.

    * `:or` — OR of enabled axis events.
    * `:and` — AND of enabled axis events.
    * `:six_d_movement` — interrupt when orientation enters a known zone.
    * `:six_d_position` — interrupt while orientation is inside a known zone.
  """
  @type aoi_mode :: :or | :and | :six_d_movement | :six_d_position

  @typedoc """
  Per-axis-direction event flags. Each entry enables interrupt generation
  when the named axis crosses the configured threshold in the named
  direction.
  """
  @type axis_event ::
          :x_high | :x_low | :y_high | :y_low | :z_high | :z_low

  @typedoc """
  Decoded `INT*_SRC` flags. `:active` is the master IA bit; the per-axis
  fields report which axis-direction events fired during the latched window
  (or this read, for non-latched mode).
  """
  @type source_flags :: %{
          active: boolean,
          x_high: boolean,
          x_low: boolean,
          y_high: boolean,
          y_low: boolean,
          z_high: boolean,
          z_low: boolean
        }

  @aoi_codes %{
    or: {0, 0},
    six_d_movement: {0, 1},
    and: {1, 0},
    six_d_position: {1, 1}
  }
  @aoi_decodes Map.new(@aoi_codes, fn {k, v} -> {v, k} end)

  @event_bits %{
    x_low: 0,
    x_high: 1,
    y_low: 2,
    y_high: 3,
    z_low: 4,
    z_high: 5
  }

  @threshold_lsb %{2 => 16, 4 => 32, 8 => 62, 16 => 186}

  @doc """
  Encode an `INT*_CFG` byte from keyword options.

  ## Options

    * `:mode` — `t:aoi_mode/0` (default `:or`).
    * `:axes` — list of `t:axis_event/0` to enable (default `[]`).
  """
  @spec encode_int_cfg(keyword) :: <<_::8>>
  def encode_int_cfg(opts \\ []) when is_list(opts) do
    mode = Keyword.get(opts, :mode, :or)
    axes = Keyword.get(opts, :axes, [])

    {aoi, six_d} = lookup!(@aoi_codes, mode, :mode)

    axis_bits =
      Enum.reduce(axes, 0, fn event, acc ->
        bit = Map.fetch!(@event_bits, event)
        acc ||| 1 <<< bit
      end)

    <<aoi <<< 7 ||| six_d <<< 6 ||| axis_bits>>
  end

  @doc "Decode an `INT*_CFG` byte into a map of its fields."
  @spec decode_int_cfg(<<_::8>>) :: %{mode: aoi_mode, axes: [axis_event]}
  def decode_int_cfg(<<byte>>) do
    aoi = byte >>> 7 &&& 1
    six_d = byte >>> 6 &&& 1
    mode = lookup!(@aoi_decodes, {aoi, six_d}, :aoi_bits)

    axes =
      for {event, bit} <- Enum.sort_by(@event_bits, &elem(&1, 1)),
          (byte >>> bit &&& 1) == 1,
          do: event

    %{mode: mode, axes: axes}
  end

  @doc "Decode an `INT*_SRC` byte into a map of its fields."
  @spec decode_int_src(<<_::8>>) :: source_flags
  def decode_int_src(<<byte>>) do
    %{
      active: (byte >>> 6 &&& 1) == 1,
      z_high: (byte >>> 5 &&& 1) == 1,
      z_low: (byte >>> 4 &&& 1) == 1,
      y_high: (byte >>> 3 &&& 1) == 1,
      y_low: (byte >>> 2 &&& 1) == 1,
      x_high: (byte >>> 1 &&& 1) == 1,
      x_low: (byte &&& 1) == 1
    }
  end

  @doc """
  Returns the threshold register's LSB size in milli-g for the given full-
  scale range, per datasheet §8.23 / §8.27.
  """
  @spec threshold_lsb_mg(Config.range()) :: pos_integer
  def threshold_lsb_mg(range), do: Map.fetch!(@threshold_lsb, range)

  @doc """
  Encode a threshold in milli-g into a 7-bit `INT*_THS` register value for
  the given range. Rounds down. Clamps at the 7-bit maximum (127).
  """
  @spec encode_threshold!(non_neg_integer, Config.range()) :: <<_::8>>
  def encode_threshold!(threshold_mg, range)
      when is_integer(threshold_mg) and threshold_mg >= 0 do
    lsb = threshold_lsb_mg(range)
    raw = min(div(threshold_mg, lsb), 0x7F)
    <<raw>>
  end

  @doc """
  Encode a duration in ODR counts into a 7-bit `INT*_DURATION` register
  value. Each LSB is `1/ODR` (so at 100 Hz, count=1 ≈ 10 ms).
  """
  @spec encode_duration!(0..127) :: <<_::8>>
  def encode_duration!(count) when is_integer(count) and count in 0..127 do
    <<count>>
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
