defmodule LIS3DH.FifoTest do
  use ExUnit.Case, async: true

  alias LIS3DH.Fifo

  describe "encode_fifo_ctrl_reg/1" do
    test "encodes Stream mode + INT1 trigger + watermark 16 → 0x8F" do
      # FM=10, TR=0, FTH=15 = 0b1000_1111 = 0x8F
      assert <<0x8F>> = Fifo.encode_fifo_ctrl_reg(mode: :stream, watermark: 16)
    end

    test "encodes Stream-to-FIFO mode + INT2 trigger + watermark 32 → 0xFF" do
      # FM=11, TR=1, FTH=31 = 0b1111_1111 = 0xFF
      assert <<0xFF>> =
               Fifo.encode_fifo_ctrl_reg(mode: :stream_to_fifo, trigger: :int2, watermark: 32)
    end

    test "encodes Bypass mode with all options at defaults → 0x0F" do
      # FM=00, TR=0, FTH=15 (watermark default 16) → 0x0F
      assert <<0x0F>> = Fifo.encode_fifo_ctrl_reg(mode: :bypass)
    end

    test "encodes FIFO mode + watermark 1" do
      # FM=01, TR=0, FTH=0 → 0b0100_0000 = 0x40
      assert <<0x40>> = Fifo.encode_fifo_ctrl_reg(mode: :fifo, watermark: 1)
    end

    test "raises on out-of-range watermark" do
      assert_raise ArgumentError, ~r/invalid watermark/, fn ->
        Fifo.encode_fifo_ctrl_reg(mode: :stream, watermark: 0)
      end

      assert_raise ArgumentError, ~r/invalid watermark/, fn ->
        Fifo.encode_fifo_ctrl_reg(mode: :stream, watermark: 33)
      end
    end

    test "raises when :mode is missing" do
      assert_raise KeyError, fn ->
        Fifo.encode_fifo_ctrl_reg(watermark: 16)
      end
    end
  end

  describe "decode_fifo_ctrl_reg/1 round-trips encode_fifo_ctrl_reg/1" do
    test "across every supported field combination" do
      for mode <- [:bypass, :fifo, :stream, :stream_to_fifo],
          trigger <- [:int1, :int2],
          watermark <- [1, 8, 16, 32] do
        opts = [mode: mode, trigger: trigger, watermark: watermark]
        decoded = opts |> Fifo.encode_fifo_ctrl_reg() |> Fifo.decode_fifo_ctrl_reg()
        assert decoded == Map.new(opts)
      end
    end
  end

  describe "decode_fifo_src_reg/1" do
    test "decodes the watermark/overrun/empty flags and stored count" do
      # WTM=1, OVRN=0, EMPTY=0, FSS=16 → 0b1001_0000 = 0x90
      assert %{watermark_reached: true, overrun: false, empty: false, stored: 16} =
               Fifo.decode_fifo_src_reg(<<0x90>>)

      # WTM=0, OVRN=1, EMPTY=0, FSS=31 (full) → 0b0101_1111 = 0x5F
      assert %{watermark_reached: false, overrun: true, empty: false, stored: 31} =
               Fifo.decode_fifo_src_reg(<<0x5F>>)

      # WTM=0, OVRN=0, EMPTY=1, FSS=0 → 0b0010_0000 = 0x20
      assert %{watermark_reached: false, overrun: false, empty: true, stored: 0} =
               Fifo.decode_fifo_src_reg(<<0x20>>)
    end
  end
end
