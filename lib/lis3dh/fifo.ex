defmodule LIS3DH.Fifo do
  @moduledoc """
  FIFO protocol encoding and decoding for the LIS3DH.

  The chip has a 32-level FIFO. Each level holds one X/Y/Z sample (6 bytes
  total, all three axes read out from `OUT_X_L..OUT_Z_H`). Four FIFO modes
  are supported, selected by `FIFO_CTRL_REG.FM[1:0]`:

    * `:bypass` — FIFO disabled. Reads of `OUT_*` return the latest sample
      live, no buffering.
    * `:fifo` — fills until full (32 samples) and stops collecting until the
      buffer is reset (by re-entering Bypass).
    * `:stream` — overwrites oldest sample once full (continuous streaming).
    * `:stream_to_fifo` — runs in Stream until an interrupt on the chosen
      trigger pin, then switches to FIFO mode (useful for capturing the
      history around an event).

  The watermark interrupt fires when the number of stored samples reaches
  `FTH[4:0] + 1`. Overrun (full) is a separate flag.

  References: *LIS3DH Datasheet DocID17530 Rev 2* §5.1 and §8.19.
  """

  import Bitwise

  @typedoc "FIFO operating mode for `FIFO_CTRL_REG.FM`."
  @type mode :: :bypass | :fifo | :stream | :stream_to_fifo

  @typedoc "Trigger pin used by Stream-to-FIFO mode (and not used by the others)."
  @type trigger :: :int1 | :int2

  @typedoc "Watermark threshold (number of stored samples that triggers the WTM flag)."
  @type watermark :: 1..32

  @typedoc "Decoded FIFO source register flags."
  @type source_flags :: %{
          watermark_reached: boolean,
          overrun: boolean,
          empty: boolean,
          stored: 0..32
        }

  @mode_codes %{bypass: 0b00, fifo: 0b01, stream: 0b10, stream_to_fifo: 0b11}
  @trigger_codes %{int1: 0, int2: 1}

  @mode_decodes Map.new(@mode_codes, fn {k, v} -> {v, k} end)
  @trigger_decodes Map.new(@trigger_codes, fn {k, v} -> {v, k} end)

  @doc """
  Encode a `FIFO_CTRL_REG` byte from keyword options.

  ## Options

    * `:mode` — `t:mode/0` (required).
    * `:trigger` — `t:trigger/0` (default `:int1`). Only meaningful in
      `:stream_to_fifo` mode; ignored otherwise.
    * `:watermark` — `t:watermark/0` (default `16`). Stored as `watermark - 1`
      in the 5-bit FTH field.
  """
  @spec encode_fifo_ctrl_reg(keyword) :: <<_::8>>
  def encode_fifo_ctrl_reg(opts) when is_list(opts) do
    mode_code = lookup!(@mode_codes, Keyword.fetch!(opts, :mode), :mode)
    trigger_code = lookup!(@trigger_codes, Keyword.get(opts, :trigger, :int1), :trigger)
    watermark = Keyword.get(opts, :watermark, 16)

    unless is_integer(watermark) and watermark in 1..32 do
      raise ArgumentError, "invalid watermark: #{inspect(watermark)} (valid: 1..32)"
    end

    fth = watermark - 1
    <<mode_code <<< 6 ||| trigger_code <<< 5 ||| fth>>
  end

  @doc "Decode a `FIFO_CTRL_REG` byte into a map of its fields."
  @spec decode_fifo_ctrl_reg(<<_::8>>) :: %{mode: mode, trigger: trigger, watermark: watermark}
  def decode_fifo_ctrl_reg(<<byte>>) do
    %{
      mode: lookup!(@mode_decodes, byte >>> 6 &&& 0b11, :mode_code),
      trigger: lookup!(@trigger_decodes, byte >>> 5 &&& 0b1, :trigger_code),
      watermark: (byte &&& 0b11111) + 1
    }
  end

  @doc "Decode the read-only `FIFO_SRC_REG` byte."
  @spec decode_fifo_src_reg(<<_::8>>) :: source_flags
  def decode_fifo_src_reg(<<byte>>) do
    %{
      watermark_reached: (byte >>> 7 &&& 1) == 1,
      overrun: (byte >>> 6 &&& 1) == 1,
      empty: (byte >>> 5 &&& 1) == 1,
      stored: byte &&& 0b11111
    }
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
