defmodule LIS3DHTest do
  use ExUnit.Case, async: true
  use Mimic

  alias Wafer.Chip
  alias Wafer.Driver.Fake
  alias Wafer.I2C

  setup :verify_on_exit!

  describe "acquire/1" do
    test "wraps a Wafer conn and verifies WHO_AM_I by default" do
      {:ok, fake} = Fake.acquire([])

      I2C
      |> expect(:write_read, fn ^fake, <<0x8F>>, 1, _opts -> {:ok, <<0x33>>, fake} end)

      assert {:ok, %LIS3DH{conn: ^fake}} = LIS3DH.acquire(conn: fake)
    end

    test "skips WHO_AM_I when :verify_who_am_i is false" do
      {:ok, fake} = Fake.acquire([])
      assert {:ok, %LIS3DH{conn: ^fake}} = LIS3DH.acquire(conn: fake, verify_who_am_i: false)
    end

    test "returns a WHO_AM_I mismatch error" do
      {:ok, fake} = Fake.acquire([])

      I2C
      |> expect(:write_read, fn ^fake, <<0x8F>>, 1, _opts -> {:ok, <<0x42>>, fake} end)

      assert {:error, {:who_am_i_mismatch, got: 0x42, expected: 0x33}} =
               LIS3DH.acquire(conn: fake)
    end

    test "reboots before WHO_AM_I when :reboot is true" do
      {:ok, fake} = Fake.acquire([])

      I2C
      # read CTRL_REG5
      |> expect(:write_read, fn ^fake, <<0xA4>>, 1, _opts -> {:ok, <<0x00>>, fake} end)
      # write CTRL_REG5 with BOOT bit set
      |> expect(:write, fn ^fake, <<0xA4, 0x80>>, _opts -> {:ok, fake} end)
      # read WHO_AM_I
      |> expect(:write_read, fn ^fake, <<0x8F>>, 1, _opts -> {:ok, <<0x33>>, fake} end)

      assert {:ok, %LIS3DH{}} = LIS3DH.acquire(conn: fake, reboot: true)
    end

    test "requires the :conn option" do
      assert {:error, _} = LIS3DH.acquire([])
    end
  end

  describe "who_am_i/1" do
    test "returns the WHO_AM_I byte" do
      {:ok, fake} = Fake.acquire([])
      acc = %LIS3DH{conn: fake}

      I2C
      |> expect(:write_read, fn ^fake, <<0x8F>>, 1, _opts -> {:ok, <<0x33>>, fake} end)

      assert {:ok, 0x33} = LIS3DH.who_am_i(acc)
    end
  end

  describe "reboot/1" do
    test "preserves the other CTRL_REG5 bits while setting BOOT" do
      {:ok, fake} = Fake.acquire([])
      acc = %LIS3DH{conn: fake}

      I2C
      |> expect(:write_read, fn ^fake, <<0xA4>>, 1, _opts -> {:ok, <<0x42>>, fake} end)
      |> expect(:write, fn ^fake, <<0xA4, 0xC2>>, _opts -> {:ok, fake} end)

      assert {:ok, %LIS3DH{}} = LIS3DH.reboot(acc)
    end
  end

  describe "configure_accelerometer/2" do
    test "writes CTRL_REG4 then CTRL_REG1 and caches mode + range" do
      {:ok, fake} = Fake.acquire([])
      acc = %LIS3DH{conn: fake}

      I2C
      # CTRL_REG4: BDU=1, FS=00, HR=1 → 0x88
      |> expect(:write, fn ^fake, <<0xA3, 0x88>>, _opts -> {:ok, fake} end)
      # CTRL_REG1: ODR=0101 (100Hz), LPen=0, axes=111 → 0x57
      |> expect(:write, fn ^fake, <<0xA0, 0x57>>, _opts -> {:ok, fake} end)

      assert {:ok, %LIS3DH{operating_mode: :high_resolution, range: 2}} =
               LIS3DH.configure_accelerometer(acc,
                 mode: :high_resolution,
                 odr: 100,
                 range: 2
               )
    end

    test "defaults range to 2g" do
      {:ok, fake} = Fake.acquire([])
      acc = %LIS3DH{conn: fake}

      I2C |> stub(:write, fn ^fake, _, _ -> {:ok, fake} end)

      assert {:ok, %LIS3DH{operating_mode: :normal, range: 2}} =
               LIS3DH.configure_accelerometer(acc, mode: :normal, odr: 100)
    end
  end

  describe "detect_configuration/1" do
    test "populates the cached mode and range from CTRL_REG1 + CTRL_REG4" do
      {:ok, fake} = Fake.acquire([])
      acc = %LIS3DH{conn: fake}

      I2C
      # CTRL_REG1: ODR=0101, LPen=0, axes=111 → 0x57
      |> expect(:write_read, fn ^fake, <<0xA0>>, 1, _opts -> {:ok, <<0x57>>, fake} end)
      # CTRL_REG4: BDU=1, FS=01 (±4g), HR=1 → 0x98
      |> expect(:write_read, fn ^fake, <<0xA3>>, 1, _opts -> {:ok, <<0x98>>, fake} end)

      assert {:ok, %LIS3DH{operating_mode: :high_resolution, range: 4}} =
               LIS3DH.detect_configuration(acc)
    end
  end

  describe "read_accelerometer/1" do
    test "returns an error when mode/range aren't cached" do
      {:ok, fake} = Fake.acquire([])
      assert {:error, :operating_mode_not_set} = LIS3DH.read_accelerometer(%LIS3DH{conn: fake})

      assert {:error, :range_not_set} =
               LIS3DH.read_accelerometer(%LIS3DH{conn: fake, operating_mode: :normal})
    end

    test "scales high-resolution + ±2g samples correctly", %{} do
      {:ok, fake} = Fake.acquire([])
      acc = %LIS3DH{conn: fake, operating_mode: :high_resolution, range: 2}

      # In 12-bit HR mode, left-justified +1g raw = +1000 mg / 1 mg-per-LSB = 1000 → left-shift 4 = 16000
      # 16000 little-endian = 0x80 0x3E? Let's verify: 16000 = 0x3E80. Bytes (LE): 0x80, 0x3E.
      # Scaled: 1000 mg * 9.80665 / 1000 = 9.80665 m/s².
      payload = <<0x80, 0x3E, 0x00, 0x00, 0x00, 0x00>>

      I2C
      |> expect(:write_read, fn ^fake, <<0xA8>>, 6, _opts -> {:ok, payload, fake} end)

      assert {:ok, %{x: x, y: y, z: z}} = LIS3DH.read_accelerometer(acc)
      assert_in_delta x, 9.80665, 1.0e-6
      assert y == +0.0
      assert z == +0.0
    end

    test "scales low-power + ±2g samples correctly" do
      {:ok, fake} = Fake.acquire([])
      acc = %LIS3DH{conn: fake, operating_mode: :low_power, range: 2}

      # In 8-bit LP mode, +1g = 1000 mg / 16 mg-per-LSB ≈ 62.5 → use 63 LSB.
      # Left-justified by 8 bits → 63 * 256 = 16128 → bytes (LE) 0x00, 0x3F.
      # Scaled: 63 * 16 mg * 9.80665 / 1000 = 9.8 m/s² (≈1g)
      payload = <<0x00, 0x3F, 0x00, 0x00, 0x00, 0x00>>

      I2C
      |> expect(:write_read, fn ^fake, <<0xA8>>, 6, _opts -> {:ok, payload, fake} end)

      assert {:ok, %{x: x}} = LIS3DH.read_accelerometer(acc)
      assert_in_delta x, 63 * 16 * 9.80665 / 1000, 1.0e-6
    end

    test "handles negative samples (sign-preserving right shift)" do
      {:ok, fake} = Fake.acquire([])
      acc = %LIS3DH{conn: fake, operating_mode: :high_resolution, range: 2}

      # -1g HR: -1000 mg / 1 = -1000 → left-shift 4 = -16000 → 0x4080 in two's
      # complement (16-bit). 16-bit signed value -16000 = 0xC180 unsigned? Let me verify.
      # Actually: -16000 in 16-bit two's complement = 65536-16000 = 49536 = 0xC180.
      # Bytes (LE): 0x80, 0xC1.
      payload = <<0x80, 0xC1, 0x00, 0x00, 0x00, 0x00>>

      I2C
      |> expect(:write_read, fn ^fake, <<0xA8>>, 6, _opts -> {:ok, payload, fake} end)

      assert {:ok, %{x: x}} = LIS3DH.read_accelerometer(acc)
      assert_in_delta x, -9.80665, 1.0e-6
    end
  end

  describe "auxiliary ADC enable/disable" do
    test "enable_auxiliary_adc sets bit 7 of TEMP_CFG_REG" do
      {:ok, fake} = Fake.acquire([])
      acc = %LIS3DH{conn: fake}

      I2C
      |> expect(:write_read, fn ^fake, <<0x9F>>, 1, _opts -> {:ok, <<0x00>>, fake} end)
      |> expect(:write, fn ^fake, <<0x9F, 0x80>>, _opts -> {:ok, fake} end)

      assert {:ok, %LIS3DH{}} = LIS3DH.enable_auxiliary_adc(acc)
    end

    test "disable_auxiliary_adc clears bit 7 of TEMP_CFG_REG" do
      {:ok, fake} = Fake.acquire([])
      acc = %LIS3DH{conn: fake}

      I2C
      |> expect(:write_read, fn ^fake, <<0x9F>>, 1, _opts -> {:ok, <<0xC0>>, fake} end)
      |> expect(:write, fn ^fake, <<0x9F, 0x40>>, _opts -> {:ok, fake} end)

      assert {:ok, %LIS3DH{}} = LIS3DH.disable_auxiliary_adc(acc)
    end

    test "enable_temperature_sensor sets both ADC_EN and TEMP_EN" do
      {:ok, fake} = Fake.acquire([])
      acc = %LIS3DH{conn: fake}

      I2C
      |> expect(:write_read, fn ^fake, <<0x9F>>, 1, _opts -> {:ok, <<0x00>>, fake} end)
      |> expect(:write, fn ^fake, <<0x9F, 0xC0>>, _opts -> {:ok, fake} end)

      assert {:ok, %LIS3DH{}} = LIS3DH.enable_temperature_sensor(acc)
    end

    test "disable_temperature_sensor clears TEMP_EN only" do
      {:ok, fake} = Fake.acquire([])
      acc = %LIS3DH{conn: fake}

      I2C
      |> expect(:write_read, fn ^fake, <<0x9F>>, 1, _opts -> {:ok, <<0xC0>>, fake} end)
      |> expect(:write, fn ^fake, <<0x9F, 0x80>>, _opts -> {:ok, fake} end)

      assert {:ok, %LIS3DH{}} = LIS3DH.disable_temperature_sensor(acc)
    end
  end

  describe "read_auxiliary_adc/2" do
    test "errors without a cached operating_mode" do
      {:ok, fake} = Fake.acquire([])

      assert {:error, :operating_mode_not_set} =
               LIS3DH.read_auxiliary_adc(%LIS3DH{conn: fake}, 1)
    end

    test "reads channel 1 at 10-bit (normal mode) and centres at 1200 mV" do
      {:ok, fake} = Fake.acquire([])
      acc = %LIS3DH{conn: fake, operating_mode: :normal, range: 2}

      # raw = 0 → 1200 mV (centre)
      I2C
      |> expect(:write_read, fn ^fake, <<0x88>>, 2, _opts -> {:ok, <<0x00, 0x00>>, fake} end)

      assert {:ok, 1200.0} = LIS3DH.read_auxiliary_adc(acc, 1)
    end

    test "reads channel 2 at the bottom of the range" do
      {:ok, fake} = Fake.acquire([])
      acc = %LIS3DH{conn: fake, operating_mode: :normal, range: 2}

      # -512 in 10-bit signed = -400 mV → 1200 - 400 = 800 mV.
      # Left-justified by 6: -512 << 6 = -32768 → 0x8000 (two's complement, LE bytes 0x00 0x80)
      I2C
      |> expect(:write_read, fn ^fake, <<0x8A>>, 2, _opts -> {:ok, <<0x00, 0x80>>, fake} end)

      assert {:ok, 800.0} = LIS3DH.read_auxiliary_adc(acc, 2)
    end

    test "reads channel 3 in low-power mode (8-bit aux ADC)" do
      {:ok, fake} = Fake.acquire([])
      acc = %LIS3DH{conn: fake, operating_mode: :low_power, range: 2}

      # raw = -128 in 8-bit signed = -400 mV → 800 mV.
      # Left-justified by 8: -128 << 8 = -32768 → 0x8000 (LE 0x00 0x80)
      I2C
      |> expect(:write_read, fn ^fake, <<0x8C>>, 2, _opts -> {:ok, <<0x00, 0x80>>, fake} end)

      assert {:ok, 800.0} = LIS3DH.read_auxiliary_adc(acc, 3)
    end
  end

  describe "read_temperature/1" do
    test "returns the signed high byte as a 1 LSB/°C delta" do
      {:ok, fake} = Fake.acquire([])
      acc = %LIS3DH{conn: fake, operating_mode: :normal, range: 2}

      # Only OUT_ADC3_H is meaningful: 0x05 → +5 °C. Low byte is junk and must be ignored.
      I2C
      |> expect(:write_read, fn ^fake, <<0x8C>>, 2, _opts -> {:ok, <<0xAA, 0x05>>, fake} end)

      assert {:ok, 5.0} = LIS3DH.read_temperature(acc)
    end

    test "handles negative deltas" do
      {:ok, fake} = Fake.acquire([])
      acc = %LIS3DH{conn: fake, operating_mode: :normal, range: 2}

      # High byte 0xF6 = -10 signed.
      I2C
      |> expect(:write_read, fn ^fake, <<0x8C>>, 2, _opts -> {:ok, <<0xAA, 0xF6>>, fake} end)

      assert {:ok, -10.0} = LIS3DH.read_temperature(acc)
    end

    test "resolution is independent of operating mode" do
      {:ok, fake} = Fake.acquire([])
      acc = %LIS3DH{conn: fake, operating_mode: :low_power, range: 2}

      I2C
      |> expect(:write_read, fn ^fake, <<0x8C>>, 2, _opts -> {:ok, <<0x00, 0x0B>>, fake} end)

      assert {:ok, 11.0} = LIS3DH.read_temperature(acc)
    end
  end

  describe "configure_free_fall/3" do
    test "writes the AND/all-axes-low pattern with default threshold + duration" do
      {:ok, fake} = Fake.acquire([])
      acc = %LIS3DH{conn: fake, operating_mode: :normal, range: 2}

      I2C
      # threshold: 350 mg / 16 mg-per-LSB = 21 → 0x15
      |> expect(:write, fn ^fake, <<0xB2, 0x15>>, _opts -> {:ok, fake} end)
      # duration: 5 → 0x05
      |> expect(:write, fn ^fake, <<0xB3, 0x05>>, _opts -> {:ok, fake} end)
      # INT1_CFG: AOI=1, 6D=0, ZL+YL+XL = 0b1001_0101 = 0x95
      |> expect(:write, fn ^fake, <<0xB0, 0x95>>, _opts -> {:ok, fake} end)

      assert {:ok, %LIS3DH{}} = LIS3DH.configure_free_fall(acc, :int1)
    end
  end

  describe "configure_motion/3" do
    test "writes the OR/all-axes-high pattern" do
      {:ok, fake} = Fake.acquire([])
      acc = %LIS3DH{conn: fake, operating_mode: :normal, range: 2}

      I2C
      # threshold: 250 / 16 = 15 → 0x0F
      |> expect(:write, fn ^fake, <<0xB6, 0x0F>>, _opts -> {:ok, fake} end)
      |> expect(:write, fn ^fake, <<0xB7, 0x00>>, _opts -> {:ok, fake} end)
      # INT2_CFG: AOI=0, 6D=0, ZH+YH+XH = 0b0010_1010 = 0x2A
      |> expect(:write, fn ^fake, <<0xB4, 0x2A>>, _opts -> {:ok, fake} end)

      assert {:ok, %LIS3DH{}} =
               LIS3DH.configure_motion(acc, :int2, threshold_mg: 250)
    end
  end

  describe "configure_orientation/3" do
    test "encodes 6D position with all axes and sets D4D_INT1 to 0 by default" do
      {:ok, fake} = Fake.acquire([])
      acc = %LIS3DH{conn: fake, operating_mode: :normal, range: 2}

      I2C
      # threshold: 320 / 16 = 20 → 0x14
      |> expect(:write, fn ^fake, <<0xB2, 0x14>>, _opts -> {:ok, fake} end)
      |> expect(:write, fn ^fake, <<0xB3, 0x00>>, _opts -> {:ok, fake} end)
      # INT1_CFG: AOI=1, 6D=1, all 6 axes = 0xFF
      |> expect(:write, fn ^fake, <<0xB0, 0xFF>>, _opts -> {:ok, fake} end)
      # CTRL_REG5 read-modify-write: clear D4D_INT1 (bit 2)
      |> expect(:write_read, fn ^fake, <<0xA4>>, 1, _opts -> {:ok, <<0x04>>, fake} end)
      |> expect(:write, fn ^fake, <<0xA4, 0x00>>, _opts -> {:ok, fake} end)

      assert {:ok, %LIS3DH{}} =
               LIS3DH.configure_orientation(acc, :int1, threshold_mg: 320)
    end

    test "4D detection sets D4D_INT2" do
      {:ok, fake} = Fake.acquire([])
      acc = %LIS3DH{conn: fake, operating_mode: :normal, range: 2}

      I2C
      |> expect(:write, fn ^fake, <<0xB6, 0x14>>, _opts -> {:ok, fake} end)
      |> expect(:write, fn ^fake, <<0xB7, 0x00>>, _opts -> {:ok, fake} end)
      # INT2_CFG: AOI=0, 6D=1, all 6 = 0b0111_1111 = 0x7F
      |> expect(:write, fn ^fake, <<0xB4, 0x7F>>, _opts -> {:ok, fake} end)
      # CTRL_REG5: set D4D_INT2 (bit 0)
      |> expect(:write_read, fn ^fake, <<0xA4>>, 1, _opts -> {:ok, <<0x00>>, fake} end)
      |> expect(:write, fn ^fake, <<0xA4, 0x01>>, _opts -> {:ok, fake} end)

      assert {:ok, %LIS3DH{}} =
               LIS3DH.configure_orientation(acc, :int2,
                 mode: :movement,
                 detection: :four_d,
                 threshold_mg: 320
               )
    end
  end

  describe "configure_click/2" do
    test "writes CLICK_THS, TIME_LIMIT, TIME_LATENCY, TIME_WINDOW, then CLICK_CFG" do
      {:ok, fake} = Fake.acquire([])
      acc = %LIS3DH{conn: fake, operating_mode: :normal, range: 2}

      I2C
      # CLICK_THS at 0x3A: 1200 mg / 16 mg-per-LSB = 75 → 0x4B (LIR=0)
      |> expect(:write, fn ^fake, <<0xBA, 0x4B>>, _opts -> {:ok, fake} end)
      # TIME_LIMIT at 0x3B: 10
      |> expect(:write, fn ^fake, <<0xBB, 0x0A>>, _opts -> {:ok, fake} end)
      # TIME_LATENCY at 0x3C: 20
      |> expect(:write, fn ^fake, <<0xBC, 0x14>>, _opts -> {:ok, fake} end)
      # TIME_WINDOW at 0x3D: 100
      |> expect(:write, fn ^fake, <<0xBD, 0x64>>, _opts -> {:ok, fake} end)
      # CLICK_CFG at 0x38: single-click Z = 0x10
      |> expect(:write, fn ^fake, <<0xB8, 0x10>>, _opts -> {:ok, fake} end)

      assert {:ok, %LIS3DH{}} =
               LIS3DH.configure_click(acc,
                 events: [:single_click_z],
                 threshold_mg: 1200,
                 time_limit: 10,
                 time_latency: 20,
                 time_window: 100
               )
    end

    test "errors when range isn't cached" do
      {:ok, fake} = Fake.acquire([])
      acc = %LIS3DH{conn: fake, range: nil}

      assert {:error, :range_not_set} =
               LIS3DH.configure_click(acc,
                 events: [],
                 threshold_mg: 0,
                 time_limit: 0,
                 time_latency: 0
               )
    end
  end

  describe "read_click_source/1" do
    test "decodes CLICK_SRC" do
      {:ok, fake} = Fake.acquire([])
      acc = %LIS3DH{conn: fake}

      I2C
      |> expect(:write_read, fn ^fake, <<0xB9>>, 1, _opts -> {:ok, <<0x64>>, fake} end)

      assert {:ok, %{double_click: true, z: true}} = LIS3DH.read_click_source(acc)
    end
  end

  describe "configure_activity/2" do
    test "writes ACT_THS then ACT_DUR" do
      {:ok, fake} = Fake.acquire([])
      acc = %LIS3DH{conn: fake, operating_mode: :low_power, range: 2}

      I2C
      # ACT_THS at 0x3E: 320 / 16 = 20 → 0x14
      |> expect(:write, fn ^fake, <<0xBE, 0x14>>, _opts -> {:ok, fake} end)
      # ACT_DUR at 0x3F: 0x0A
      |> expect(:write, fn ^fake, <<0xBF, 0x0A>>, _opts -> {:ok, fake} end)

      assert {:ok, %LIS3DH{}} =
               LIS3DH.configure_activity(acc, threshold_mg: 320, duration: 10)
    end
  end

  describe "disable_activity/1" do
    test "writes 0 to ACT_THS" do
      {:ok, fake} = Fake.acquire([])
      acc = %LIS3DH{conn: fake}

      I2C
      |> expect(:write, fn ^fake, <<0xBE, 0x00>>, _opts -> {:ok, fake} end)

      assert {:ok, %LIS3DH{}} = LIS3DH.disable_activity(acc)
    end
  end

  describe "configure_inertial_interrupt/3" do
    test "writes INT1_THS, INT1_DURATION, INT1_CFG in that order using the cached range" do
      {:ok, fake} = Fake.acquire([])
      acc = %LIS3DH{conn: fake, operating_mode: :normal, range: 2}

      I2C
      # INT1_THS = 160 mg / 16 mg-per-LSB = 10 → 0x0A at 0x32
      |> expect(:write, fn ^fake, <<0xB2, 0x0A>>, _opts -> {:ok, fake} end)
      # INT1_DURATION = 5 → 0x05 at 0x33
      |> expect(:write, fn ^fake, <<0xB3, 0x05>>, _opts -> {:ok, fake} end)
      # INT1_CFG = AOI(1)|6D(0)|XYZ-low → 0b1001_0101 = 0x95 at 0x30
      |> expect(:write, fn ^fake, <<0xB0, 0x95>>, _opts -> {:ok, fake} end)

      assert {:ok, %LIS3DH{}} =
               LIS3DH.configure_inertial_interrupt(acc, :int1,
                 mode: :and,
                 axes: [:x_low, :y_low, :z_low],
                 threshold_mg: 160,
                 duration: 5
               )
    end

    test "errors when range isn't cached" do
      {:ok, fake} = Fake.acquire([])
      acc = %LIS3DH{conn: fake, operating_mode: :normal, range: nil}

      assert {:error, :range_not_set} =
               LIS3DH.configure_inertial_interrupt(acc, :int1, threshold_mg: 0)
    end
  end

  describe "read_interrupt_source/2" do
    test "reads INT2_SRC and decodes the flags" do
      {:ok, fake} = Fake.acquire([])
      acc = %LIS3DH{conn: fake}

      I2C
      |> expect(:write_read, fn ^fake, <<0xB5>>, 1, _opts -> {:ok, <<0x46>>, fake} end)

      assert {:ok, %{active: true, z_high: false, y_low: true, x_high: true}} =
               LIS3DH.read_interrupt_source(acc, :int2)
    end
  end

  describe "interrupt routing" do
    test "enable_int1_routing ORs in :ia1 + :click" do
      {:ok, fake} = Fake.acquire([])
      acc = %LIS3DH{conn: fake}

      # CTRL_REG3 at 0x22; mask for :ia1 + :click = 0xC0
      I2C
      |> expect(:write_read, fn ^fake, <<0xA2>>, 1, _opts -> {:ok, <<0x04>>, fake} end)
      |> expect(:write, fn ^fake, <<0xA2, 0xC4>>, _opts -> {:ok, fake} end)

      assert {:ok, %LIS3DH{}} = LIS3DH.enable_int1_routing(acc, [:ia1, :click])
    end

    test "disable_int1_routing masks out specified bits, leaves others" do
      {:ok, fake} = Fake.acquire([])
      acc = %LIS3DH{conn: fake}

      I2C
      |> expect(:write_read, fn ^fake, <<0xA2>>, 1, _opts -> {:ok, <<0xC4>>, fake} end)
      |> expect(:write, fn ^fake, <<0xA2, 0x84>>, _opts -> {:ok, fake} end)

      assert {:ok, %LIS3DH{}} = LIS3DH.disable_int1_routing(acc, [:ia1])
    end

    test "enable_int2_routing ORs in :activity + :ia2" do
      {:ok, fake} = Fake.acquire([])
      acc = %LIS3DH{conn: fake}

      # CTRL_REG6 at 0x25; mask :activity (bit 3) + :ia2 (bit 5) = 0x28
      I2C
      |> expect(:write_read, fn ^fake, <<0xA5>>, 1, _opts -> {:ok, <<0x00>>, fake} end)
      |> expect(:write, fn ^fake, <<0xA5, 0x28>>, _opts -> {:ok, fake} end)

      assert {:ok, %LIS3DH{}} = LIS3DH.enable_int2_routing(acc, [:activity, :ia2])
    end
  end

  describe "set_interrupt_polarity/2" do
    test "active_low sets bit 1 of CTRL_REG6" do
      {:ok, fake} = Fake.acquire([])
      acc = %LIS3DH{conn: fake}

      I2C
      |> expect(:write_read, fn ^fake, <<0xA5>>, 1, _opts -> {:ok, <<0x28>>, fake} end)
      |> expect(:write, fn ^fake, <<0xA5, 0x2A>>, _opts -> {:ok, fake} end)

      assert {:ok, %LIS3DH{}} = LIS3DH.set_interrupt_polarity(acc, :active_low)
    end
  end

  describe "set_interrupt_latching/3 and set_4d_detection/3" do
    test "latching INT1 sets bit 3 of CTRL_REG5" do
      {:ok, fake} = Fake.acquire([])
      acc = %LIS3DH{conn: fake}

      I2C
      |> expect(:write_read, fn ^fake, <<0xA4>>, 1, _opts -> {:ok, <<0x00>>, fake} end)
      |> expect(:write, fn ^fake, <<0xA4, 0x08>>, _opts -> {:ok, fake} end)

      assert {:ok, %LIS3DH{}} = LIS3DH.set_interrupt_latching(acc, :int1, true)
    end

    test "4D on INT2 sets bit 0 of CTRL_REG5" do
      {:ok, fake} = Fake.acquire([])
      acc = %LIS3DH{conn: fake}

      I2C
      |> expect(:write_read, fn ^fake, <<0xA4>>, 1, _opts -> {:ok, <<0x00>>, fake} end)
      |> expect(:write, fn ^fake, <<0xA4, 0x01>>, _opts -> {:ok, fake} end)

      assert {:ok, %LIS3DH{}} = LIS3DH.set_4d_detection(acc, :int2, true)
    end
  end

  describe "set_self_test/2" do
    test "sets ST field to 01 while preserving the other CTRL_REG4 bits" do
      {:ok, fake} = Fake.acquire([])
      acc = %LIS3DH{conn: fake}

      I2C
      # current CTRL_REG4: BDU=1 + HR=1 = 0x88
      |> expect(:write_read, fn ^fake, <<0xA3>>, 1, _opts -> {:ok, <<0x88>>, fake} end)
      # new CTRL_REG4: same bits + ST=01 (bit 1) = 0x8A
      |> expect(:write, fn ^fake, <<0xA3, 0x8A>>, _opts -> {:ok, fake} end)

      assert {:ok, %LIS3DH{}} = LIS3DH.set_self_test(acc, :self_test_0)
    end

    test "clears ST field to 00" do
      {:ok, fake} = Fake.acquire([])
      acc = %LIS3DH{conn: fake}

      I2C
      |> expect(:write_read, fn ^fake, <<0xA3>>, 1, _opts -> {:ok, <<0x8A>>, fake} end)
      |> expect(:write, fn ^fake, <<0xA3, 0x88>>, _opts -> {:ok, fake} end)

      assert {:ok, %LIS3DH{}} = LIS3DH.set_self_test(acc, :off)
    end

    test "sets ST field to 10" do
      {:ok, fake} = Fake.acquire([])
      acc = %LIS3DH{conn: fake}

      I2C
      |> expect(:write_read, fn ^fake, <<0xA3>>, 1, _opts -> {:ok, <<0x88>>, fake} end)
      |> expect(:write, fn ^fake, <<0xA3, 0x8C>>, _opts -> {:ok, fake} end)

      assert {:ok, %LIS3DH{}} = LIS3DH.set_self_test(acc, :self_test_1)
    end
  end

  describe "configure_high_pass_filter/2" do
    test "writes CTRL_REG2" do
      {:ok, fake} = Fake.acquire([])
      acc = %LIS3DH{conn: fake}

      I2C
      |> expect(:write, fn ^fake, <<0xA1, 0x8F>>, _opts -> {:ok, fake} end)

      assert {:ok, %LIS3DH{}} =
               LIS3DH.configure_high_pass_filter(acc,
                 mode: :normal,
                 filtered_data_output: true,
                 enable_for_click: true,
                 enable_for_interrupt_1: true,
                 enable_for_interrupt_2: true
               )
    end
  end

  describe "REFERENCE register helpers" do
    test "read_reference returns a signed byte" do
      {:ok, fake} = Fake.acquire([])
      acc = %LIS3DH{conn: fake}

      I2C
      |> expect(:write_read, fn ^fake, <<0xA6>>, 1, _opts -> {:ok, <<0xFF>>, fake} end)

      assert {:ok, -1} = LIS3DH.read_reference(acc)
    end

    test "write_reference writes a signed byte" do
      {:ok, fake} = Fake.acquire([])
      acc = %LIS3DH{conn: fake}

      I2C
      |> expect(:write, fn ^fake, <<0xA6, 0xFE>>, _opts -> {:ok, fake} end)

      assert {:ok, %LIS3DH{}} = LIS3DH.write_reference(acc, -2)
    end
  end

  describe "power_on/2 and power_off/1" do
    test "power_on sets the ODR field while preserving the LPen and axis bits" do
      {:ok, fake} = Fake.acquire([])
      acc = %LIS3DH{conn: fake}

      I2C
      # current CTRL_REG1: 0x0F = power-down + axes enabled
      |> expect(:write_read, fn ^fake, <<0xA0>>, 1, _opts -> {:ok, <<0x0F>>, fake} end)
      # new CTRL_REG1: ODR=0101 (100 Hz), keep low 4 bits → 0x5F
      |> expect(:write, fn ^fake, <<0xA0, 0x5F>>, _opts -> {:ok, fake} end)

      assert {:ok, %LIS3DH{}} = LIS3DH.power_on(acc, 100)
    end

    test "power_off zeros the ODR field while preserving the low 4 bits" do
      {:ok, fake} = Fake.acquire([])
      acc = %LIS3DH{conn: fake}

      I2C
      |> expect(:write_read, fn ^fake, <<0xA0>>, 1, _opts -> {:ok, <<0x5F>>, fake} end)
      |> expect(:write, fn ^fake, <<0xA0, 0x0F>>, _opts -> {:ok, fake} end)

      assert {:ok, %LIS3DH{}} = LIS3DH.power_off(acc)
    end
  end

  describe "Wafer.Chip.read_register/3" do
    test "ORs 0x80 into the sub-address for auto-increment" do
      {:ok, fake} = Fake.acquire([])
      acc = %LIS3DH{conn: fake}

      I2C
      |> expect(:write_read, fn ^fake, <<0x8F>>, 1, _opts ->
        # 0x0F (WHO_AM_I) | 0x80 = 0x8F
        {:ok, <<0x33>>, fake}
      end)

      assert {:ok, <<0x33>>} = Chip.read_register(acc, 0x0F, 1)
    end

    test "supports multi-byte burst reads" do
      {:ok, fake} = Fake.acquire([])
      acc = %LIS3DH{conn: fake}
      sample = <<0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC>>

      I2C
      |> expect(:write_read, fn ^fake, <<0xA8>>, 6, _opts ->
        # 0x28 (OUT_X_L) | 0x80 = 0xA8
        {:ok, sample, fake}
      end)

      assert {:ok, ^sample} = Chip.read_register(acc, 0x28, 6)
    end

    test "propagates transport errors" do
      {:ok, fake} = Fake.acquire([])
      acc = %LIS3DH{conn: fake}

      I2C
      |> expect(:write_read, fn _, _, _, _ -> {:error, :i2c_nak} end)

      assert {:error, :i2c_nak} = Chip.read_register(acc, 0x0F, 1)
    end

    test "rejects out-of-range addresses" do
      {:ok, fake} = Fake.acquire([])
      acc = %LIS3DH{conn: fake}

      assert {:error, _} = Chip.read_register(acc, -1, 1)
      assert {:error, _} = Chip.read_register(acc, 0x80, 1)
    end

    test "rejects zero byte counts" do
      {:ok, fake} = Fake.acquire([])
      acc = %LIS3DH{conn: fake}

      assert {:error, _} = Chip.read_register(acc, 0x0F, 0)
    end
  end

  describe "Wafer.Chip.write_register/3" do
    test "ORs 0x80 into the sub-address and writes data" do
      {:ok, fake} = Fake.acquire([])
      acc = %LIS3DH{conn: fake}

      I2C
      |> expect(:write, fn ^fake, <<0xA0, 0x57>>, _opts -> {:ok, fake} end)

      assert {:ok, %LIS3DH{}} = Chip.write_register(acc, 0x20, <<0x57>>)
    end

    test "supports multi-byte bursts" do
      {:ok, fake} = Fake.acquire([])
      acc = %LIS3DH{conn: fake}

      I2C
      |> expect(:write, fn ^fake, <<0xA0, 0x57, 0x00, 0x08>>, _opts -> {:ok, fake} end)

      assert {:ok, %LIS3DH{}} = Chip.write_register(acc, 0x20, <<0x57, 0x00, 0x08>>)
    end

    test "rejects out-of-range addresses" do
      {:ok, fake} = Fake.acquire([])
      acc = %LIS3DH{conn: fake}

      assert {:error, _} = Chip.write_register(acc, 0x80, <<0x00>>)
    end

    test "rejects empty data" do
      {:ok, fake} = Fake.acquire([])
      acc = %LIS3DH{conn: fake}

      assert {:error, _} = Chip.write_register(acc, 0x20, <<>>)
    end
  end

  describe "Wafer.Chip.swap_register/3" do
    test "reads the old value then writes the new one" do
      {:ok, fake} = Fake.acquire([])
      acc = %LIS3DH{conn: fake}

      I2C
      |> expect(:write_read, fn ^fake, <<0xA0>>, 1, _opts -> {:ok, <<0x00>>, fake} end)
      |> expect(:write, fn ^fake, <<0xA0, 0x57>>, _opts -> {:ok, fake} end)

      assert {:ok, <<0x00>>, %LIS3DH{}} = Chip.swap_register(acc, 0x20, <<0x57>>)
    end
  end
end
