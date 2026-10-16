-- =========================================================================
-- Signals & Systems / DSP Practical Labs - Task 3 (v2, VHDL)
-- File    : test_tone_dds.vhd
-- Purpose : Built-in digital test signal (Direct Digital Synthesis).
--           A 32-bit phase accumulator advances once per sample tick and
--           addresses a 1024-entry 12-bit sine ROM (sine_lut_pkg, generated
--           by python/gen_sine_lut.py).  Frequency is chosen with SW[3:1].
--           Used to verify the FPGA -> ESP32 -> PC data path and the FFT
--           frequency axis without any analog source.
-- =========================================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.sine_lut_pkg.all;

entity test_tone_dds is
  port (
    clk      : in  std_logic;
    rst      : in  std_logic;
    tick     : in  std_logic;                       -- sample tick (Fs)
    freq_sel : in  std_logic_vector(2 downto 0);
    data     : out std_logic_vector(11 downto 0);
    valid    : out std_logic
  );
end entity test_tone_dds;

architecture rtl of test_tone_dds is
  signal phase   : unsigned(DDS_ACC_BITS - 1 downto 0) := (others => '0');
  signal addr    : unsigned(LUT_ADDR_BITS - 1 downto 0) := (others => '0');
  signal v_d     : std_logic := '0';
  signal data_r  : std_logic_vector(11 downto 0) := x"800";
  signal valid_r : std_logic := '0';
begin
  process (clk)
    variable inc : unsigned(DDS_ACC_BITS - 1 downto 0);
  begin
    if rising_edge(clk) then
      inc := to_unsigned(TONE_PHASE_INC(to_integer(unsigned(freq_sel))), DDS_ACC_BITS);
      v_d     <= '0';
      valid_r <= v_d;
      if rst = '1' then
        phase <= (others => '0');
        addr  <= (others => '0');
      elsif tick = '1' then
        addr  <= phase(DDS_ACC_BITS - 1 downto DDS_ACC_BITS - LUT_ADDR_BITS);
        phase <= phase + inc;
        v_d   <= '1';
      end if;
      -- registered ROM read (maps to M9K block RAM)
      data_r <= std_logic_vector(to_unsigned(SINE_LUT(to_integer(addr)), 12));
    end if;
  end process;

  data  <= data_r;
  valid <= valid_r;
end architecture rtl;
