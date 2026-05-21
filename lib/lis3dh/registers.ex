defmodule LIS3DH.Registers do
  @moduledoc """
  Raw register accessors for the LIS3DH.

  Each function reads or writes a single 8-bit register. Higher-level helpers
  in `LIS3DH` and friends interpret the bytes as signed integers, scaled
  physical quantities, or named bit-fields.

  Register addresses, access modes and bit layouts are taken from
  *LIS3DH Datasheet DocID17530 Rev 2* §7 *Register mapping* and §8
  *Registers description*.
  """

  use Wafer.Registers

  # Status / auxiliary ADC channels
  defregister(:status_reg_aux, 0x07, :ro, 1)
  defregister(:out_adc1_l, 0x08, :ro, 1)
  defregister(:out_adc1_h, 0x09, :ro, 1)
  defregister(:out_adc2_l, 0x0A, :ro, 1)
  defregister(:out_adc2_h, 0x0B, :ro, 1)
  defregister(:out_adc3_l, 0x0C, :ro, 1)
  defregister(:out_adc3_h, 0x0D, :ro, 1)

  # Identification
  defregister(:who_am_i, 0x0F, :ro, 1)

  # Control
  defregister(:ctrl_reg_0, 0x1E, :rw, 1)
  defregister(:temp_cfg_reg, 0x1F, :rw, 1)
  defregister(:ctrl_reg_1, 0x20, :rw, 1)
  defregister(:ctrl_reg_2, 0x21, :rw, 1)
  defregister(:ctrl_reg_3, 0x22, :rw, 1)
  defregister(:ctrl_reg_4, 0x23, :rw, 1)
  defregister(:ctrl_reg_5, 0x24, :rw, 1)
  defregister(:ctrl_reg_6, 0x25, :rw, 1)
  defregister(:reference, 0x26, :rw, 1)

  # Accelerometer status + data
  defregister(:status_reg, 0x27, :ro, 1)
  defregister(:out_x_l, 0x28, :ro, 1)
  defregister(:out_x_h, 0x29, :ro, 1)
  defregister(:out_y_l, 0x2A, :ro, 1)
  defregister(:out_y_h, 0x2B, :ro, 1)
  defregister(:out_z_l, 0x2C, :ro, 1)
  defregister(:out_z_h, 0x2D, :ro, 1)

  # FIFO
  defregister(:fifo_ctrl_reg, 0x2E, :rw, 1)
  defregister(:fifo_src_reg, 0x2F, :ro, 1)

  # Inertial interrupt 1
  defregister(:int1_cfg, 0x30, :rw, 1)
  defregister(:int1_src, 0x31, :ro, 1)
  defregister(:int1_ths, 0x32, :rw, 1)
  defregister(:int1_duration, 0x33, :rw, 1)

  # Inertial interrupt 2
  defregister(:int2_cfg, 0x34, :rw, 1)
  defregister(:int2_src, 0x35, :ro, 1)
  defregister(:int2_ths, 0x36, :rw, 1)
  defregister(:int2_duration, 0x37, :rw, 1)

  # Click / tap detection
  defregister(:click_cfg, 0x38, :rw, 1)
  defregister(:click_src, 0x39, :ro, 1)
  defregister(:click_ths, 0x3A, :rw, 1)
  defregister(:time_limit, 0x3B, :rw, 1)
  defregister(:time_latency, 0x3C, :rw, 1)
  defregister(:time_window, 0x3D, :rw, 1)

  # Activity / sleep-to-wake
  defregister(:act_ths, 0x3E, :rw, 1)
  defregister(:act_dur, 0x3F, :rw, 1)
end
