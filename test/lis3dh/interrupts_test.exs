defmodule LIS3DH.InterruptsTest do
  use ExUnit.Case, async: true

  alias LIS3DH.Interrupts

  describe "encode_int_cfg/1" do
    test "defaults to all zeros (no events, OR mode)" do
      assert <<0x00>> = Interrupts.encode_int_cfg()
    end

    test "encodes AOI=AND with all 6 axis-low events for free-fall" do
      # AOI=1, 6D=0, axes = X/Y/Z low → bits 0, 2, 4 set
      # 0b1001_0101 = 0x95
      assert <<0x95>> =
               Interrupts.encode_int_cfg(
                 mode: :and,
                 axes: [:x_low, :y_low, :z_low]
               )
    end

    test "encodes AOI=OR with all 6 axis-high events for motion / wake-up" do
      # AOI=0, 6D=0, axes = X/Y/Z high → bits 1, 3, 5 set
      # 0b0010_1010 = 0x2A
      assert <<0x2A>> =
               Interrupts.encode_int_cfg(
                 mode: :or,
                 axes: [:x_high, :y_high, :z_high]
               )
    end

    test "encodes 6D movement recognition" do
      # AOI=0, 6D=1, axes=all-high → 0b0110_1010 = 0x6A
      assert <<0x6A>> =
               Interrupts.encode_int_cfg(
                 mode: :six_d_movement,
                 axes: [:x_high, :y_high, :z_high]
               )
    end

    test "encodes 6D position recognition" do
      # AOI=1, 6D=1, axes=all 6 → 0b1111_1111 = 0xFF
      assert <<0xFF>> =
               Interrupts.encode_int_cfg(
                 mode: :six_d_position,
                 axes: [:x_high, :x_low, :y_high, :y_low, :z_high, :z_low]
               )
    end

    test "raises on unknown mode" do
      assert_raise ArgumentError, ~r/invalid mode/, fn ->
        Interrupts.encode_int_cfg(mode: :wat)
      end
    end
  end

  describe "decode_int_cfg/1 round-trips encode_int_cfg/1" do
    test "for representative configurations" do
      configs = [
        [mode: :and, axes: [:x_low, :y_low, :z_low]],
        [mode: :or, axes: [:x_high, :y_high, :z_high]],
        [mode: :six_d_movement, axes: [:x_high, :z_low]],
        [mode: :six_d_position, axes: []],
        [mode: :or, axes: []]
      ]

      for opts <- configs do
        decoded = opts |> Interrupts.encode_int_cfg() |> Interrupts.decode_int_cfg()
        assert decoded.mode == Keyword.fetch!(opts, :mode)
        assert Enum.sort(decoded.axes) == Enum.sort(Keyword.fetch!(opts, :axes))
      end
    end
  end

  describe "decode_int_src/1" do
    test "decodes the active flag and per-axis-direction bits" do
      # IA=1, ZH=1, ZL=0, YH=0, YL=1, XH=1, XL=0 → 0b0110_0110 = 0x66
      assert %{
               active: true,
               z_high: true,
               z_low: false,
               y_high: false,
               y_low: true,
               x_high: true,
               x_low: false
             } =
               Interrupts.decode_int_src(<<0x66>>)
    end
  end

  describe "threshold_lsb_mg/1" do
    test "returns the documented LSB sizes per range" do
      assert Interrupts.threshold_lsb_mg(2) == 16
      assert Interrupts.threshold_lsb_mg(4) == 32
      assert Interrupts.threshold_lsb_mg(8) == 62
      assert Interrupts.threshold_lsb_mg(16) == 186
    end
  end

  describe "encode_threshold!/2" do
    test "rounds down to the nearest LSB" do
      assert <<10>> = Interrupts.encode_threshold!(160, 2)
      assert <<10>> = Interrupts.encode_threshold!(175, 2)
      assert <<11>> = Interrupts.encode_threshold!(176, 2)
    end

    test "clamps at the 7-bit maximum (127)" do
      assert <<0x7F>> = Interrupts.encode_threshold!(100_000, 2)
    end

    test "0 mg encodes to 0" do
      assert <<0>> = Interrupts.encode_threshold!(0, 8)
    end
  end

  describe "encode_duration!/1" do
    test "passes the 7-bit value through unchanged" do
      assert <<5>> = Interrupts.encode_duration!(5)
      assert <<0x7F>> = Interrupts.encode_duration!(127)
    end

    test "rejects out-of-range counts" do
      assert_raise FunctionClauseError, fn -> Interrupts.encode_duration!(128) end
      assert_raise FunctionClauseError, fn -> Interrupts.encode_duration!(-1) end
    end
  end
end
