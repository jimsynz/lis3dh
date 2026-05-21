defmodule LIS3DH.Sampler do
  @moduledoc """
  GenServer that drains the LIS3DH's FIFO and dispatches frames to a
  subscriber process.

  ## Why this exists

  Polling raw `OUT_*` registers from the BEAM is fine at low ODRs, but at
  100 Hz+ the chip's 32-level FIFO and watermark interrupt let the host
  off-load buffering to silicon and only intervene on a chosen watermark
  threshold.

  ## Modes

  Set the `:mode` option to one of:

    * `:stream` (default) — overwrites the oldest sample when full. Best for
      continuous streaming at the watermark interval. Recommended.
    * `:fifo` — fills to 32 samples and stops. Use when you want a fixed
      snapshot triggered by external timing.
    * `:stream_to_fifo` — starts in Stream and switches to FIFO when an
      interrupt fires on the configured trigger pin. Use to capture history
      around an event (motion, free-fall, click).

  The sampler only routes the FIFO watermark interrupt to **INT1** —
  `CTRL_REG3.I1_WTM`. The chip has no equivalent INT2 bit, so external
  wiring must use INT1 for FIFO.

  ## Frame format

  Each FIFO sample is one X/Y/Z accelerometer reading. Frames are dispatched
  as `{LIS3DH.Sampler, sampler_pid, [%{x: float, y: float, z: float}, ...]}`
  messages where each map contains values in m/s² scaled from the cached
  operating mode and range on the `LIS3DH` struct.

  ## Usage

      {:ok, i2c} = Wafer.Driver.Circuits.I2C.acquire(bus_name: "i2c-1", address: 0x18)
      {:ok, acc} = LIS3DH.acquire(conn: i2c)
      {:ok, acc} = LIS3DH.configure_accelerometer(acc,
        mode: :high_resolution, odr: 200, range: 2)

      {:ok, int1} = Wafer.Driver.Circuits.GPIO.acquire(pin: 17, direction: :in)

      {:ok, _sampler} =
        LIS3DH.Sampler.start_link(acc: acc, int1: int1, mode: :stream, watermark: 16)

      receive do
        {LIS3DH.Sampler, _pid, frames} -> IO.inspect(frames)
      end
  """

  use GenServer

  import Bitwise

  alias LIS3DH.Config
  alias LIS3DH.Fifo
  alias LIS3DH.Registers
  alias Wafer.Chip
  alias Wafer.Conn
  alias Wafer.GPIO

  @gravity_ms2 9.80665
  @out_x_l 0x28
  @ctrl_reg_3_i1_wtm_bit 2
  @ctrl_reg_5_fifo_en_bit 6
  @bypass_byte <<0x00>>

  @typedoc "Options accepted by `start_link/1`."
  @type option ::
          {:acc, LIS3DH.t()}
          | {:int1, Conn.t() | nil}
          | {:subscriber, pid}
          | {:mode, Fifo.mode()}
          | {:watermark, Fifo.watermark()}
          | {:trigger, Fifo.trigger()}
          | {:name, GenServer.name()}

  defstruct [:acc, :int1, :subscriber, :mode, :watermark]

  @typedoc "Internal sampler state."
  @type t :: %__MODULE__{
          acc: LIS3DH.t(),
          int1: Conn.t() | nil,
          subscriber: pid,
          mode: Fifo.mode(),
          watermark: Fifo.watermark()
        }

  @doc """
  Start a FIFO sampler.

  ## Options

    * `:acc` (required) — an `LIS3DH` struct with `:operating_mode` and
      `:range` cached (e.g. via `LIS3DH.configure_accelerometer/2`).
    * `:int1` (optional) — a `Wafer.GPIO`-implementing connection wired to
      the device's INT1 pin. Without it the sampler runs pull-only.
    * `:subscriber` (default `self()`) — process that receives frame messages.
    * `:mode` (default `:stream`) — `t:LIS3DH.Fifo.mode/0`.
    * `:watermark` (default `16`) — `t:LIS3DH.Fifo.watermark/0`.
    * `:trigger` (default `:int1`) — Stream-to-FIFO trigger pin
      (`t:LIS3DH.Fifo.trigger/0`); ignored in other modes.
    * `:name` — optional GenServer registration name.
  """
  @spec start_link([option]) :: GenServer.on_start()
  def start_link(opts) do
    {name, opts} = Keyword.pop(opts, :name)
    gen_opts = if name, do: [name: name], else: []
    GenServer.start_link(__MODULE__, opts, gen_opts)
  end

  @doc """
  Synchronously drain the FIFO and return any complete frames. Used in
  pull-only mode or for on-demand reads.
  """
  @spec drain(GenServer.server()) :: {:ok, [LIS3DH.axes()]} | {:error, term}
  def drain(server), do: GenServer.call(server, :drain)

  @impl GenServer
  def init(opts) do
    acc = Keyword.fetch!(opts, :acc)
    int1 = Keyword.get(opts, :int1)
    subscriber = Keyword.get(opts, :subscriber, self())
    mode = Keyword.get(opts, :mode, :stream)
    watermark = Keyword.get(opts, :watermark, 16)
    trigger = Keyword.get(opts, :trigger, :int1)

    with :ok <- check_configuration(acc),
         {:ok, acc} <- reset_fifo(acc),
         {:ok, acc} <- enable_fifo(acc),
         {:ok, acc} <- write_fifo_mode(acc, mode, watermark, trigger),
         {:ok, acc} <- route_watermark_interrupt(acc, int1),
         {:ok, int1} <- enable_int1_interrupt(int1) do
      {:ok,
       %__MODULE__{
         acc: acc,
         int1: int1,
         subscriber: subscriber,
         mode: mode,
         watermark: watermark
       }}
    else
      {:error, reason} -> {:stop, reason}
    end
  end

  @impl GenServer
  def handle_call(:drain, _from, state) do
    case do_drain(state) do
      {:ok, frames, state} -> {:reply, {:ok, frames}, state}
      {:error, reason, state} -> {:reply, {:error, reason}, state}
    end
  end

  @impl GenServer
  def handle_info({:interrupt, _conn, _condition, _meta}, state) do
    case do_drain(state) do
      {:ok, [], state} ->
        {:noreply, state}

      {:ok, frames, state} ->
        send(state.subscriber, {__MODULE__, self(), frames})
        {:noreply, state}

      {:error, _reason, state} ->
        {:noreply, state}
    end
  end

  def handle_info(_other, state), do: {:noreply, state}

  @impl GenServer
  def terminate(_reason, state) do
    _ = if state.int1, do: GPIO.disable_interrupt(state.int1, :rising), else: :ok
    _ = unroute_watermark_interrupt(state.acc)
    _ = Registers.write_fifo_ctrl_reg(state.acc, @bypass_byte)
    _ = disable_fifo(state.acc)
    :ok
  end

  defp check_configuration(%LIS3DH{operating_mode: nil}), do: {:error, :operating_mode_not_set}
  defp check_configuration(%LIS3DH{range: nil}), do: {:error, :range_not_set}
  defp check_configuration(%LIS3DH{}), do: :ok

  defp reset_fifo(acc) do
    # Cycling through Bypass mode is required to reset the FIFO pointer
    # between mode changes — per datasheet §5.1.1.
    Registers.write_fifo_ctrl_reg(acc, @bypass_byte)
  end

  defp enable_fifo(acc) do
    Registers.update_ctrl_reg_5(acc, fn <<byte>> -> <<byte ||| 1 <<< @ctrl_reg_5_fifo_en_bit>> end)
  end

  defp disable_fifo(acc) do
    Registers.update_ctrl_reg_5(acc, fn <<byte>> ->
      <<byte &&& bnot(1 <<< @ctrl_reg_5_fifo_en_bit) &&& 0xFF>>
    end)
  end

  defp write_fifo_mode(acc, mode, watermark, trigger) do
    Registers.write_fifo_ctrl_reg(
      acc,
      Fifo.encode_fifo_ctrl_reg(mode: mode, watermark: watermark, trigger: trigger)
    )
  end

  defp route_watermark_interrupt(acc, nil), do: {:ok, acc}

  defp route_watermark_interrupt(acc, _int1) do
    Registers.update_ctrl_reg_3(acc, fn <<byte>> ->
      <<byte ||| 1 <<< @ctrl_reg_3_i1_wtm_bit>>
    end)
  end

  defp unroute_watermark_interrupt(acc) do
    Registers.update_ctrl_reg_3(acc, fn <<byte>> ->
      <<byte &&& bnot(1 <<< @ctrl_reg_3_i1_wtm_bit) &&& 0xFF>>
    end)
  end

  defp enable_int1_interrupt(nil), do: {:ok, nil}

  defp enable_int1_interrupt(int1) do
    case GPIO.enable_interrupt(int1, :rising, nil) do
      {:ok, int1} -> {:ok, int1}
      other -> other
    end
  end

  defp do_drain(state) do
    with {:ok, src} <- Registers.read_fifo_src_reg(state.acc),
         %{stored: stored} = Fifo.decode_fifo_src_reg(src),
         frames_to_read = max(stored, 0),
         {:ok, frames} <- read_frames(state.acc, frames_to_read) do
      {:ok, frames, state}
    else
      {:error, reason} -> {:error, reason, state}
    end
  end

  defp read_frames(_acc, 0), do: {:ok, []}

  defp read_frames(acc, count) do
    bytes = count * 6

    with {:ok, data} <- Chip.read_register(acc, @out_x_l, bytes) do
      {:ok, parse_frames(data, acc, [])}
    end
  end

  defp parse_frames(<<>>, _acc, frames), do: Enum.reverse(frames)

  defp parse_frames(
         <<x::little-signed-16, y::little-signed-16, z::little-signed-16, rest::binary>>,
         acc,
         frames
       ) do
    frame = %{
      x: scale(x, acc),
      y: scale(y, acc),
      z: scale(z, acc)
    }

    parse_frames(rest, acc, [frame | frames])
  end

  defp scale(raw, %LIS3DH{operating_mode: mode, range: range}) do
    shift = 16 - Config.native_width(mode)
    sensitivity_mg = Config.sensitivity(mode, range)
    (raw >>> shift) * sensitivity_mg * @gravity_ms2 / 1000
  end
end
