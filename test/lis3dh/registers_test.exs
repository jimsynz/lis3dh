defmodule LIS3DH.RegistersTest do
  use ExUnit.Case, async: true
  use Mimic

  import Bitwise

  alias LIS3DH.Registers
  alias Wafer.Driver.Fake
  alias Wafer.I2C

  setup_all do
    Code.ensure_loaded!(LIS3DH.Registers)
    :ok
  end

  setup :verify_on_exit!

  setup do
    {:ok, fake} = Fake.acquire([])
    {:ok, acc: %LIS3DH{conn: fake}, fake: fake}
  end

  describe ":ro registers" do
    test "read_who_am_i reads from 0x0F", %{acc: acc, fake: fake} do
      I2C
      |> expect(:write_read, fn ^fake, <<0x8F>>, 1, _opts -> {:ok, <<0x33>>, fake} end)

      assert {:ok, <<0x33>>} = Registers.read_who_am_i(acc)
    end

    test "read_status_reg reads from 0x27", %{acc: acc, fake: fake} do
      I2C
      |> expect(:write_read, fn ^fake, <<0xA7>>, 1, _opts -> {:ok, <<0x0F>>, fake} end)

      assert {:ok, <<0x0F>>} = Registers.read_status_reg(acc)
    end

    test "no write helper is exported for :ro registers" do
      refute function_exported?(LIS3DH.Registers, :write_who_am_i, 2)
      refute function_exported?(LIS3DH.Registers, :write_status_reg, 2)
      refute function_exported?(LIS3DH.Registers, :write_fifo_src_reg, 2)
    end
  end

  describe ":rw registers" do
    test "read_ctrl_reg_1 reads from 0x20", %{acc: acc, fake: fake} do
      I2C
      |> expect(:write_read, fn ^fake, <<0xA0>>, 1, _opts -> {:ok, <<0x57>>, fake} end)

      assert {:ok, <<0x57>>} = Registers.read_ctrl_reg_1(acc)
    end

    test "write_ctrl_reg_1 writes to 0x20", %{acc: acc, fake: fake} do
      I2C
      |> expect(:write, fn ^fake, <<0xA0, 0x57>>, _opts -> {:ok, fake} end)

      assert {:ok, %LIS3DH{}} = Registers.write_ctrl_reg_1(acc, <<0x57>>)
    end

    test "update_fifo_ctrl_reg round-trips the callback", %{acc: acc, fake: fake} do
      I2C
      |> expect(:write_read, fn ^fake, <<0xAE>>, 1, _opts -> {:ok, <<0x00>>, fake} end)
      |> expect(:write, fn ^fake, <<0xAE, 0x80>>, _opts -> {:ok, fake} end)

      assert {:ok, %LIS3DH{}} =
               Registers.update_fifo_ctrl_reg(acc, fn <<v>> -> <<v ||| 0x80>> end)
    end
  end

  describe "register coverage spot-checks" do
    test "all documented public registers are declared", %{acc: acc, fake: fake} do
      addresses = [
        {:status_reg_aux, 0x07},
        {:out_adc1_l, 0x08},
        {:out_adc1_h, 0x09},
        {:out_adc2_l, 0x0A},
        {:out_adc2_h, 0x0B},
        {:out_adc3_l, 0x0C},
        {:out_adc3_h, 0x0D},
        {:who_am_i, 0x0F},
        {:ctrl_reg_0, 0x1E},
        {:temp_cfg_reg, 0x1F},
        {:ctrl_reg_1, 0x20},
        {:ctrl_reg_2, 0x21},
        {:ctrl_reg_3, 0x22},
        {:ctrl_reg_4, 0x23},
        {:ctrl_reg_5, 0x24},
        {:ctrl_reg_6, 0x25},
        {:reference, 0x26},
        {:status_reg, 0x27},
        {:out_x_l, 0x28},
        {:out_x_h, 0x29},
        {:out_y_l, 0x2A},
        {:out_y_h, 0x2B},
        {:out_z_l, 0x2C},
        {:out_z_h, 0x2D},
        {:fifo_ctrl_reg, 0x2E},
        {:fifo_src_reg, 0x2F},
        {:int1_cfg, 0x30},
        {:int1_src, 0x31},
        {:int1_ths, 0x32},
        {:int1_duration, 0x33},
        {:int2_cfg, 0x34},
        {:int2_src, 0x35},
        {:int2_ths, 0x36},
        {:int2_duration, 0x37},
        {:click_cfg, 0x38},
        {:click_src, 0x39},
        {:click_ths, 0x3A},
        {:time_limit, 0x3B},
        {:time_latency, 0x3C},
        {:time_window, 0x3D},
        {:act_ths, 0x3E},
        {:act_dur, 0x3F}
      ]

      I2C
      |> stub(:write_read, fn ^fake, <<sub_addr>>, 1, _ ->
        {:ok, <<bxor(sub_addr, 0x80)>>, fake}
      end)

      for {name, address} <- addresses do
        fun = :"read_#{name}"
        assert function_exported?(LIS3DH.Registers, fun, 1), "missing read_#{name}/1"
        expected = address
        assert {:ok, <<^expected>>} = apply(LIS3DH.Registers, fun, [acc])
      end
    end
  end
end
