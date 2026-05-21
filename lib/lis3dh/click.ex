defmodule LIS3DH.Click do
  @moduledoc """
  Encoding and decoding for the LIS3DH's click / double-click / tap
  detection engine.

  Six per-axis enable bits in `CLICK_CFG` select which axes and click types
  (single vs double) are armed. Timing is configured by three registers:

    * `TIME_LIMIT` — maximum pulse width for a click (must rise above the
      threshold and fall back within this window).
    * `TIME_LATENCY` — dead time after a click during which another pulse
      is ignored.
    * `TIME_WINDOW` — the additional window after the latency in which a
      second click is allowed (for double-click detection).

  All three timing values are in `1/ODR` units.

  References: *LIS3DH Datasheet DocID17530 Rev 2* §8.29 – §8.34.
  """

  import Bitwise

  alias LIS3DH.Interrupts

  @typedoc """
  Per-axis / click-type enables for `CLICK_CFG`.
  """
  @type click_event ::
          :single_click_x
          | :double_click_x
          | :single_click_y
          | :double_click_y
          | :single_click_z
          | :double_click_z

  @typedoc "Decoded `CLICK_SRC` flags."
  @type source_flags :: %{
          active: boolean,
          double_click: boolean,
          single_click: boolean,
          sign: :positive | :negative,
          x: boolean,
          y: boolean,
          z: boolean
        }

  @event_bits %{
    single_click_x: 0,
    double_click_x: 1,
    single_click_y: 2,
    double_click_y: 3,
    single_click_z: 4,
    double_click_z: 5
  }

  @doc """
  Encode a `CLICK_CFG` byte from a list of `t:click_event/0` to enable.
  """
  @spec encode_click_cfg([click_event]) :: <<_::8>>
  def encode_click_cfg(events) when is_list(events) do
    value =
      Enum.reduce(events, 0, fn event, acc ->
        bit = Map.fetch!(@event_bits, event)
        acc ||| 1 <<< bit
      end)

    <<value>>
  end

  @doc "Decode a `CLICK_CFG` byte into a list of enabled `t:click_event/0`."
  @spec decode_click_cfg(<<_::8>>) :: [click_event]
  def decode_click_cfg(<<byte>>) do
    for {event, bit} <- Enum.sort_by(@event_bits, &elem(&1, 1)),
        (byte >>> bit &&& 1) == 1,
        do: event
  end

  @doc """
  Encode a `CLICK_THS` byte from a threshold in milli-g (using the same LSB
  table as the inertial interrupt thresholds) and a latch-request flag.
  """
  @spec encode_click_ths!(non_neg_integer, LIS3DH.Config.range(), boolean) :: <<_::8>>
  def encode_click_ths!(threshold_mg, range, latched?)
      when is_integer(threshold_mg) and threshold_mg >= 0 do
    <<raw>> = Interrupts.encode_threshold!(threshold_mg, range)
    lir = if latched?, do: 1 <<< 7, else: 0
    <<lir ||| raw>>
  end

  @doc "Decode a `CLICK_SRC` byte into a map of its fields."
  @spec decode_click_src(<<_::8>>) :: source_flags
  def decode_click_src(<<byte>>) do
    %{
      active: (byte >>> 6 &&& 1) == 1,
      double_click: (byte >>> 5 &&& 1) == 1,
      single_click: (byte >>> 4 &&& 1) == 1,
      sign: if((byte >>> 3 &&& 1) == 1, do: :negative, else: :positive),
      z: (byte >>> 2 &&& 1) == 1,
      y: (byte >>> 1 &&& 1) == 1,
      x: (byte &&& 1) == 1
    }
  end
end
