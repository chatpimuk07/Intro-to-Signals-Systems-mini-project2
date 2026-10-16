-- =========================================================================
-- Signals & Systems / DSP Practical Labs - Task 3 (v2, VHDL)
-- File    : sample_tick_gen.vhd
-- Purpose : Hardware sampling clock-enable.
--           Produces a single-cycle 'tick' every CLK_HZ/FS_HZ clock cycles
--           (50 MHz / 16 kHz = 3125 cycles -> Ts = 62.5 us exactly).
--           'tick_toggle' flips on every tick -> FS/2 square wave (8 kHz)
--           for oscilloscope verification of the sampling rate.
--           The counter is free running and depends on nothing but the
--           crystal clock, so data transfer can never delay a sample.
-- =========================================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity sample_tick_gen is
  generic (
    CLK_HZ : positive := 50_000_000;
    FS_HZ  : positive := 16_000
  );
  port (
    clk         : in  std_logic;
    rst         : in  std_logic;     -- synchronous, active high
    tick        : out std_logic;     -- 1-clock pulse at FS_HZ
    tick_toggle : out std_logic      -- FS_HZ/2 square wave
  );
end entity sample_tick_gen;

architecture rtl of sample_tick_gen is
  constant DIV : positive := CLK_HZ / FS_HZ;
  signal cnt    : natural range 0 to DIV - 1 := 0;
  signal tick_r : std_logic := '0';
  signal tog_r  : std_logic := '0';
begin
  assert (CLK_HZ mod FS_HZ) = 0
    report "sample_tick_gen: CLK_HZ is not an integer multiple of FS_HZ - Fs will be rounded"
    severity warning;

  process (clk)
  begin
    if rising_edge(clk) then
      if rst = '1' then
        cnt    <= 0;
        tick_r <= '0';
        tog_r  <= '0';
      elsif cnt = DIV - 1 then
        cnt    <= 0;
        tick_r <= '1';
        tog_r  <= not tog_r;
      else
        cnt    <= cnt + 1;
        tick_r <= '0';
      end if;
    end if;
  end process;

  tick        <= tick_r;
  tick_toggle <= tog_r;
end architecture rtl;
