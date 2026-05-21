defmodule LIS3DH.ClickTest do
  use ExUnit.Case, async: true

  alias LIS3DH.Click

  describe "encode_click_cfg/1" do
    test "encodes all six events as 0x3F" do
      assert <<0x3F>> =
               Click.encode_click_cfg([
                 :single_click_x,
                 :double_click_x,
                 :single_click_y,
                 :double_click_y,
                 :single_click_z,
                 :double_click_z
               ])
    end

    test "encodes single-click Z only as 0x10" do
      assert <<0x10>> = Click.encode_click_cfg([:single_click_z])
    end

    test "encodes empty list as 0x00" do
      assert <<0x00>> = Click.encode_click_cfg([])
    end
  end

  describe "decode_click_cfg/1 round-trips encode_click_cfg/1" do
    test "across representative selections" do
      selections = [
        [:single_click_x],
        [:double_click_y],
        [:single_click_x, :double_click_z],
        [:single_click_x, :single_click_y, :single_click_z],
        []
      ]

      for events <- selections do
        decoded = events |> Click.encode_click_cfg() |> Click.decode_click_cfg()
        assert Enum.sort(decoded) == Enum.sort(events)
      end
    end
  end

  describe "encode_click_ths!/3" do
    test "sets LIR_Click bit when latched? is true" do
      # 160 mg @ ±2g (16 mg/LSB) = 10 = 0x0A. LIR = 0x80. Combined = 0x8A
      assert <<0x8A>> = Click.encode_click_ths!(160, 2, true)
    end

    test "clears LIR_Click bit when latched? is false" do
      assert <<0x0A>> = Click.encode_click_ths!(160, 2, false)
    end
  end

  describe "decode_click_src/1" do
    test "decodes a positive double-click on the Z axis" do
      # IA=1, DCLICK=1, SCLICK=0, Sign=0 (positive), Z=1, Y=0, X=0
      # 0b0110_0100 = 0x64
      assert %{
               active: true,
               double_click: true,
               single_click: false,
               sign: :positive,
               z: true,
               y: false,
               x: false
             } = Click.decode_click_src(<<0x64>>)
    end

    test "decodes a negative single-click on the X axis" do
      # IA=1, DCLICK=0, SCLICK=1, Sign=1 (negative), Z=0, Y=0, X=1
      # 0b0101_1001 = 0x59
      assert %{
               active: true,
               single_click: true,
               sign: :negative,
               x: true,
               y: false,
               z: false
             } = Click.decode_click_src(<<0x59>>)
    end
  end
end
