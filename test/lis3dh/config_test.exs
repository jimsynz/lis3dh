defmodule LIS3DH.ConfigTest do
  use ExUnit.Case, async: true

  alias LIS3DH.Config

  describe "encode_ctrl_reg_1/1" do
    test "encodes normal mode + 100 Hz + all axes = 0x57" do
      # ODR=0101, LPen=0, Zen=1, Yen=1, Xen=1 → 0b0101_0111 = 0x57
      assert <<0x57>> = Config.encode_ctrl_reg_1(mode: :normal, odr: 100)
    end

    test "encodes low-power mode + 1.6 kHz + all axes = 0x8F" do
      # ODR=1000, LPen=1, axes=111 → 0b1000_1111 = 0x8F
      assert <<0x8F>> = Config.encode_ctrl_reg_1(mode: :low_power, odr: 1600)
    end

    test "encodes high-resolution mode + 50 Hz + Z only = 0x44" do
      # ODR=0100, LPen=0, Zen=1, Yen=0, Xen=0 → 0b0100_0100 = 0x44
      assert <<0x44>> = Config.encode_ctrl_reg_1(mode: :high_resolution, odr: 50, axes: [:z])
    end

    test "power_down encodes ODR field to all zeros" do
      assert <<0x07>> = Config.encode_ctrl_reg_1(mode: :normal, odr: :power_down)
    end

    test "1344 and 5376 share ODR code 1001" do
      assert <<0x97>> = Config.encode_ctrl_reg_1(mode: :normal, odr: 1344)
      assert <<0x9F>> = Config.encode_ctrl_reg_1(mode: :low_power, odr: 5376)
    end

    test "raises on unknown odr" do
      assert_raise ArgumentError, ~r/invalid odr/, fn ->
        Config.encode_ctrl_reg_1(mode: :normal, odr: 999)
      end
    end
  end

  describe "encode_ctrl_reg_4/1" do
    test "encodes BDU=hold + ±2g + normal = 0x80" do
      # BDU=1, FS=00, HR=0 → 0b1000_0000 = 0x80
      assert <<0x80>> = Config.encode_ctrl_reg_4(mode: :normal)
    end

    test "encodes BDU=hold + ±8g + high-resolution = 0xA8" do
      # BDU=1, FS=10, HR=1 → 0b1010_1000 = 0xA8
      assert <<0xA8>> = Config.encode_ctrl_reg_4(mode: :high_resolution, range: 8)
    end

    test "encodes BDU=continuous + ±16g + low-power = 0x30" do
      # BDU=0, FS=11, HR=0 → 0b0011_0000 = 0x30
      assert <<0x30>> =
               Config.encode_ctrl_reg_4(
                 mode: :low_power,
                 range: 16,
                 block_data_update: :continuous
               )
    end
  end

  describe "decode_ctrl_reg_1/1 + decode_ctrl_reg_4/1 round-trip" do
    test "every combination" do
      for mode <- [:low_power, :normal, :high_resolution],
          range <- [2, 4, 8, 16],
          odr <- [1, 10, 25, 50, 100, 200, 400],
          axes <- [[:x, :y, :z], [:x], [:y, :z]] do
        ctrl_reg_1 = Config.encode_ctrl_reg_1(mode: mode, odr: odr, axes: axes)
        ctrl_reg_4 = Config.encode_ctrl_reg_4(mode: mode, range: range)

        %{lpen: lpen, odr: ^odr, axes: ^axes} = Config.decode_ctrl_reg_1(ctrl_reg_1)
        %{hr: hr, range: ^range, block_data_update: :hold} = Config.decode_ctrl_reg_4(ctrl_reg_4)

        assert Config.operating_mode(lpen, hr) == mode
      end
    end
  end

  describe "operating_mode/2" do
    test "rejects the disallowed combination" do
      assert_raise ArgumentError, ~r/LPen=1 \+ HR=1/, fn ->
        Config.operating_mode(true, true)
      end
    end
  end

  describe "sensitivity/2" do
    test "returns the published mg/LSB values" do
      assert Config.sensitivity(:high_resolution, 2) == 1
      assert Config.sensitivity(:high_resolution, 16) == 12
      assert Config.sensitivity(:normal, 2) == 4
      assert Config.sensitivity(:normal, 16) == 48
      assert Config.sensitivity(:low_power, 2) == 16
      assert Config.sensitivity(:low_power, 16) == 192
    end
  end

  describe "native_width/1" do
    test "returns the bit width per mode" do
      assert Config.native_width(:low_power) == 8
      assert Config.native_width(:normal) == 10
      assert Config.native_width(:high_resolution) == 12
    end
  end
end
