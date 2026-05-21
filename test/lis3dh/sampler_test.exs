defmodule LIS3DH.SamplerTest do
  use ExUnit.Case, async: false
  use Mimic

  alias LIS3DH.Sampler
  alias Wafer.Driver.Fake
  alias Wafer.GPIO
  alias Wafer.I2C

  setup :set_mimic_global
  setup :verify_on_exit!

  setup do
    {:ok, i2c} = Fake.acquire([])
    {:ok, int1} = Fake.acquire([])

    acc = %LIS3DH{conn: i2c, operating_mode: :high_resolution, range: 2}

    {:ok, i2c: i2c, int1: int1, acc: acc}
  end

  describe "start_link/1" do
    test "writes bypass-reset → enables FIFO → writes mode/watermark → routes WTM to INT1", %{
      i2c: i2c,
      int1: int1,
      acc: acc
    } do
      I2C
      # 1) FIFO_CTRL_REG ← 0x00 (bypass reset)
      |> expect(:write, fn ^i2c, <<0xAE, 0x00>>, _ -> {:ok, i2c} end)
      # 2) CTRL_REG5 read-modify-write to set FIFO_EN
      |> expect(:write_read, fn ^i2c, <<0xA4>>, 1, _ -> {:ok, <<0x00>>, i2c} end)
      |> expect(:write, fn ^i2c, <<0xA4, 0x40>>, _ -> {:ok, i2c} end)
      # 3) FIFO_CTRL_REG ← Stream + INT1 trig + watermark 16 = 0x8F
      |> expect(:write, fn ^i2c, <<0xAE, 0x8F>>, _ -> {:ok, i2c} end)
      # 4) CTRL_REG3 read-modify-write to set I1_WTM (bit 2)
      |> expect(:write_read, fn ^i2c, <<0xA2>>, 1, _ -> {:ok, <<0x00>>, i2c} end)
      |> expect(:write, fn ^i2c, <<0xA2, 0x04>>, _ -> {:ok, i2c} end)

      GPIO
      |> expect(:enable_interrupt, fn ^int1, :rising, _ -> {:ok, int1} end)
      |> stub(:disable_interrupt, fn ^int1, _ -> {:ok, int1} end)

      I2C |> stub(:write, fn ^i2c, _, _ -> {:ok, i2c} end)
      I2C |> stub(:write_read, fn ^i2c, _, _, _ -> {:ok, <<0x00>>, i2c} end)

      assert {:ok, pid} = Sampler.start_link(acc: acc, int1: int1)
      GenServer.stop(pid)
    end

    test "refuses to start when operating_mode isn't cached", %{int1: int1} do
      {:ok, fake} = Fake.acquire([])
      acc = %LIS3DH{conn: fake, operating_mode: nil, range: 2}

      Process.flag(:trap_exit, true)
      assert {:error, :operating_mode_not_set} = Sampler.start_link(acc: acc, int1: int1)
    end

    test "refuses to start when range isn't cached", %{int1: int1} do
      {:ok, fake} = Fake.acquire([])
      acc = %LIS3DH{conn: fake, operating_mode: :normal, range: nil}

      Process.flag(:trap_exit, true)
      assert {:error, :range_not_set} = Sampler.start_link(acc: acc, int1: int1)
    end

    test "skips INT1 routing when no GPIO conn is provided", %{i2c: i2c, acc: acc} do
      I2C |> stub(:write, fn ^i2c, _, _ -> {:ok, i2c} end)
      I2C |> stub(:write_read, fn ^i2c, _, _, _ -> {:ok, <<0x00>>, i2c} end)

      reject(&GPIO.enable_interrupt/3)

      assert {:ok, pid} = Sampler.start_link(acc: acc)
      GenServer.stop(pid)
    end
  end

  describe "drain/1" do
    setup %{i2c: i2c, int1: int1, acc: acc} do
      I2C |> stub(:write, fn ^i2c, _, _ -> {:ok, i2c} end)
      I2C |> stub(:write_read, fn ^i2c, _, _, _ -> {:ok, <<0x00>>, i2c} end)
      GPIO |> stub(:enable_interrupt, fn ^int1, _, _ -> {:ok, int1} end)
      GPIO |> stub(:disable_interrupt, fn ^int1, _ -> {:ok, int1} end)

      {:ok, sampler} = Sampler.start_link(acc: acc, int1: int1)
      on_exit(fn -> if Process.alive?(sampler), do: GenServer.stop(sampler) end)
      {:ok, sampler: sampler}
    end

    test "returns an empty list when FIFO_SRC reports EMPTY", %{i2c: i2c, sampler: sampler} do
      I2C
      |> expect(:write_read, fn ^i2c, <<0xAF>>, 1, _ -> {:ok, <<0x20>>, i2c} end)

      assert {:ok, []} = Sampler.drain(sampler)
    end

    test "drains and scales the configured FSS count", %{i2c: i2c, sampler: sampler} do
      # One frame at +1g X: HR mode, ±2g → 1000 LSB at 12-bit → left-shift 4 → 16000 = 0x3E80
      frame = <<0x80, 0x3E, 0x00, 0x00, 0x00, 0x00>>

      I2C
      |> expect(:write_read, fn ^i2c, <<0xAF>>, 1, _ -> {:ok, <<0x01>>, i2c} end)
      |> expect(:write_read, fn ^i2c, <<0xA8>>, 6, _ -> {:ok, frame, i2c} end)

      assert {:ok, [%{x: x, y: y, z: z}]} = Sampler.drain(sampler)
      assert_in_delta x, 9.80665, 1.0e-6
      assert y == +0.0
      assert z == +0.0
    end
  end

  describe "interrupt-driven dispatch" do
    setup %{i2c: i2c, int1: int1, acc: acc} do
      I2C |> stub(:write, fn ^i2c, _, _ -> {:ok, i2c} end)
      I2C |> stub(:write_read, fn ^i2c, _, _, _ -> {:ok, <<0x00>>, i2c} end)
      GPIO |> stub(:enable_interrupt, fn ^int1, _, _ -> {:ok, int1} end)
      GPIO |> stub(:disable_interrupt, fn ^int1, _ -> {:ok, int1} end)

      {:ok, sampler} = Sampler.start_link(acc: acc, int1: int1, subscriber: self())
      on_exit(fn -> if Process.alive?(sampler), do: GenServer.stop(sampler) end)
      {:ok, sampler: sampler}
    end

    test "sends frames on rising-edge interrupts", %{i2c: i2c, int1: int1, sampler: sampler} do
      frame = <<0x80, 0x3E, 0x00, 0x00, 0x00, 0x00>>

      I2C
      |> expect(:write_read, fn ^i2c, <<0xAF>>, 1, _ -> {:ok, <<0x01>>, i2c} end)
      |> expect(:write_read, fn ^i2c, <<0xA8>>, 6, _ -> {:ok, frame, i2c} end)

      send(sampler, {:interrupt, int1, :rising, nil})

      assert_receive {Sampler, ^sampler, [%{x: x}]}, 1_000
      assert_in_delta x, 9.80665, 1.0e-6
    end

    test "doesn't dispatch on an empty FIFO", %{i2c: i2c, int1: int1, sampler: sampler} do
      I2C
      |> expect(:write_read, fn ^i2c, <<0xAF>>, 1, _ -> {:ok, <<0x20>>, i2c} end)

      send(sampler, {:interrupt, int1, :rising, nil})
      refute_receive {Sampler, _, _}, 100
      assert Process.alive?(sampler)
    end
  end
end
