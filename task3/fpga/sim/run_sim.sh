#!/usr/bin/env bash
# Run the Task 3 (v2) self-checking testbench with nvc or GHDL (VHDL-93).
#   macOS : brew install nvc        Linux : apt install ghdl  (or nvc)
set -euo pipefail
cd "$(dirname "$0")"
RTL=../rtl
SRC=(
  "$RTL/sine_lut_pkg.vhd"
  "$RTL/sample_tick_gen.vhd"
  "$RTL/mcp3201_ctrl.vhd"
  "$RTL/max10_adc_sampler.vhd"
  "$RTL/test_tone_dds.vhd"
  "$RTL/pingpong_buffer.vhd"
  "$RTL/spi_slave_tx.vhd"
  "$RTL/task3_top.vhd"
  tb_task3_top.vhd
)
mkdir -p work_sim
if command -v nvc >/dev/null 2>&1; then
  nvc --std=1993 --work=work_sim/work -a "${SRC[@]}" -e tb_task3_top -r --exit-severity=error
elif command -v ghdl >/dev/null 2>&1; then
  ghdl -a --std=93c --workdir=work_sim "${SRC[@]}"
  ghdl -e --std=93c --workdir=work_sim tb_task3_top
  ghdl -r --std=93c --workdir=work_sim tb_task3_top --assert-level=error
else
  echo "Install nvc (brew install nvc) or GHDL first." >&2
  exit 1
fi
