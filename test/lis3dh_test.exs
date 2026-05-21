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
    test "errors without a cached operating_mode" do
      {:ok, fake} = Fake.acquire([])
      assert {:error, :operating_mode_not_set} = LIS3DH.read_temperature(%LIS3DH{conn: fake})
    end

    test "returns 1 digit/°C delta in normal mode (10-bit aux ADC)" do
      {:ok, fake} = Fake.acquire([])
      acc = %LIS3DH{conn: fake, operating_mode: :normal, range: 2}

      # raw = 5 (at 10-bit) → 5 °C delta. Left-justified by 6: 5 << 6 = 320 = 0x0140 → LE 0x40 0x01
      I2C
      |> expect(:write_read, fn ^fake, <<0x8C>>, 2, _opts -> {:ok, <<0x40, 0x01>>, fake} end)

      assert {:ok, 5.0} = LIS3DH.read_temperature(acc)
    end

    test "handles negative deltas" do
      {:ok, fake} = Fake.acquire([])
      acc = %LIS3DH{conn: fake, operating_mode: :normal, range: 2}

      # raw = -10 (at 10-bit). -10 << 6 = -640. In 16-bit two's complement = 65536-640=64896=0xFD80
      # LE: 0x80 0xFD
      I2C
      |> expect(:write_read, fn ^fake, <<0x8C>>, 2, _opts -> {:ok, <<0x80, 0xFD>>, fake} end)

      assert {:ok, -10.0} = LIS3DH.read_temperature(acc)
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
